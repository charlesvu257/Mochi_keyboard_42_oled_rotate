/*
 * NINO — flash-backed note store
 *
 * Layout of nino_partition:
 *
 *   0x00  16 bytes  header
 *   0x10  ...       blob
 *
 * The header is written LAST, at commit, and only if the blob checked out.
 * Until then those first 16 bytes are still erased, so an interrupted
 * transfer — a yanked cable, a reset mid-send — leaves a store that reads as
 * empty rather than one that reads as valid but is truncated. Flash gives us
 * that for free: erased bytes are 0xFF, which can never be a valid magic.
 *
 * SPDX-License-Identifier: MIT
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include "nino_store.h"

#define NINO_PARTITION_ID FIXED_PARTITION_ID(nino_partition)

#define STORE_MAGIC 0x4F4E494EU /* "NINO" */
#define STORE_VERSION 1U
#define HEADER_SIZE 16U

/*
 * Staging buffer. nRF52840 flash writes must be word-aligned and word-sized,
 * but blobs arrive in whatever lengths the host chose, so bytes are held here
 * until there is a whole aligned run to commit.
 */
#define STAGE_SIZE 128U

struct store_header {
    uint32_t magic;
    uint32_t version;
    uint32_t length;
    uint32_t crc;
};

static const struct flash_area *store_area;

static struct {
    bool valid;
    uint32_t length;
    uint32_t crc;
} committed;

/*
 * Reads leave no session behind them, so "is a read happening" is a question
 * about recency rather than state. READ_IDLE_MS is how long a gap can be
 * before destow is considered finished or given up, either way something the
 * panel should stop reflecting.
 *
 * Declared here, ahead of nino_store_begin(), which clears it — a struct
 * defined after its first use does not compile.
 */
#define READ_IDLE_MS 600

static struct {
    int64_t last_ms;
    size_t high_water;
} reading;

static struct {
    bool active;
    uint32_t declared_total;
    uint32_t declared_crc;
    uint32_t received;
    uint32_t running_crc;
    size_t write_offset; /* partition-relative, always word-aligned */
    uint8_t stage[STAGE_SIZE];
    size_t staged;
} incoming;

static int store_open(void) {
    if (store_area != NULL) {
        return 0;
    }
    return flash_area_open(NINO_PARTITION_ID, &store_area);
}

static void load_header(void) {
    struct store_header hdr;

    committed.valid = false;

    if (store_open() != 0) {
        return;
    }
    if (flash_area_read(store_area, 0, &hdr, sizeof(hdr)) != 0) {
        return;
    }
    if (hdr.magic != STORE_MAGIC || hdr.version != STORE_VERSION) {
        return;
    }
    if (hdr.length > nino_store_capacity()) {
        return;
    }

    committed.valid = true;
    committed.length = hdr.length;
    committed.crc = hdr.crc;
}

static int store_init(void) {
    if (store_open() != 0) {
        LOG_ERR("NINO store: cannot open partition");
        return -EIO;
    }
    load_header();
    return 0;
}

SYS_INIT(store_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

size_t nino_store_capacity(void) {
    if (store_open() != 0) {
        return 0;
    }
    return store_area->fa_size - HEADER_SIZE;
}

size_t nino_store_used(void) { return committed.valid ? committed.length : 0; }

uint32_t nino_store_crc(void) { return committed.valid ? committed.crc : 0; }

uint32_t nino_store_expected_crc(void) { return incoming.declared_crc; }

int nino_store_erase(void) {
    int rc = store_open();
    if (rc != 0) {
        return rc;
    }

    rc = flash_area_erase(store_area, 0, store_area->fa_size);
    if (rc == 0) {
        committed.valid = false;
        incoming.active = false;
    }
    return rc;
}

int nino_store_begin(uint32_t total, uint32_t expected_crc) {
    int rc = store_open();
    if (rc != 0) {
        return rc;
    }
    if (total > nino_store_capacity()) {
        return -ENOSPC;
    }

    rc = nino_store_erase();
    if (rc != 0) {
        return rc;
    }

    /* A new blob makes any previous read's progress meaningless. */
    reading.last_ms = 0;
    reading.high_water = 0;

    incoming.active = true;
    incoming.declared_total = total;
    incoming.declared_crc = expected_crc;
    incoming.received = 0;
    incoming.running_crc = 0;
    incoming.write_offset = HEADER_SIZE;
    incoming.staged = 0;

    return 0;
}

/* Push whole aligned runs out of the staging buffer, keeping any remainder. */
static int stage_flush(bool final) {
    if (incoming.staged == 0) {
        return 0;
    }

    size_t align = flash_area_align(store_area);
    size_t writable = incoming.staged - (incoming.staged % align);

    if (final) {
        /* Round the tail up and pad with erased-state bytes. Overshooting into
         * already-erased flash is harmless; the header records the true
         * length, so the padding is never handed back. */
        while (incoming.staged % align) {
            incoming.stage[incoming.staged++] = 0xFF;
        }
        writable = incoming.staged;
    }

    if (writable == 0) {
        return 0;
    }

    int rc = flash_area_write(store_area, incoming.write_offset, incoming.stage, writable);
    if (rc != 0) {
        return rc;
    }

    incoming.write_offset += writable;
    incoming.staged -= writable;

    if (incoming.staged > 0) {
        memmove(incoming.stage, incoming.stage + writable, incoming.staged);
    }

    return 0;
}

int nino_store_append(const uint8_t *data, size_t len) {
    if (!incoming.active) {
        return -EINVAL;
    }
    if (incoming.received + len > incoming.declared_total) {
        return -ENOSPC;
    }

    incoming.running_crc = crc32_ieee_update(incoming.running_crc, data, len);
    incoming.received += len;

    while (len > 0) {
        size_t room = STAGE_SIZE - incoming.staged;
        size_t take = MIN(room, len);

        memcpy(incoming.stage + incoming.staged, data, take);
        incoming.staged += take;
        data += take;
        len -= take;

        if (incoming.staged == STAGE_SIZE) {
            int rc = stage_flush(false);
            if (rc != 0) {
                return rc;
            }
        }
    }

    return 0;
}

int nino_store_commit(uint32_t *actual_crc) {
    if (!incoming.active) {
        return -EINVAL;
    }

    incoming.active = false;

    if (actual_crc != NULL) {
        *actual_crc = incoming.running_crc;
    }

    /* Refuse to write a header the blob does not deserve. */
    if (incoming.received != incoming.declared_total ||
        incoming.running_crc != incoming.declared_crc) {
        return -EBADMSG;
    }

    int rc = stage_flush(true);
    if (rc != 0) {
        return rc;
    }

    struct store_header hdr = {
        .magic = STORE_MAGIC,
        .version = STORE_VERSION,
        .length = incoming.received,
        .crc = incoming.running_crc,
    };

    rc = flash_area_write(store_area, 0, &hdr, sizeof(hdr));
    if (rc != 0) {
        return rc;
    }

    committed.valid = true;
    committed.length = hdr.length;
    committed.crc = hdr.crc;

    return 0;
}

bool nino_store_busy(void) { return incoming.active; }

bool nino_store_reading(void) {
    return reading.last_ms != 0 && (k_uptime_get() - reading.last_ms) < READ_IDLE_MS;
}

uint8_t nino_store_read_progress(void) {
    if (!committed.valid || committed.length == 0) {
        return 0;
    }
    return (uint8_t)(((uint64_t)reading.high_water * 255U) / committed.length);
}

uint8_t nino_store_progress(void) {
    if (!incoming.active || incoming.declared_total == 0) {
        return 0;
    }
    return (uint8_t)(((uint64_t)incoming.received * 255U) / incoming.declared_total);
}

int nino_store_read(size_t offset, uint8_t *buf, size_t len) {
    if (!committed.valid) {
        return -ENOENT;
    }
    if (offset + len > committed.length) {
        return -EINVAL;
    }

    reading.last_ms = k_uptime_get();
    if (offset + len > reading.high_water) {
        reading.high_water = offset + len;
    }

    return flash_area_read(store_area, HEADER_SIZE + offset, buf, len);
}
