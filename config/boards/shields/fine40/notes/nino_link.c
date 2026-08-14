/*
 * NINO — host link
 *
 * A line-based ASCII protocol on its own CDC ACM port. The host talks, the
 * keyboard answers, one line each:
 *
 *   HELLO                          -> NINO 1 <capacity> <used> <crc32hex>
 *   ERASE                          -> OK
 *   BEGIN <total> <crc32hex>       -> OK | FULL | ERR
 *   DATA <base32>                  -> OK | BAD
 *   END                            -> OK | BAD <expected> <actual>
 *   READ <offset> <len>            -> DATA <base32> | ERR
 *
 * READ is how notes come back off the keyboard. The host learns the length
 * from HELLO and walks the blob itself, which keeps the firmware free of any
 * streaming state and makes a failed chunk retryable on its own.
 *
 * Blob bytes travel as base32 rather than raw, so the whole protocol is
 * printable and line-delimited and there is no framing to get wrong. The 1.6x
 * cost on the wire is irrelevant — USB is not the scarce resource here, flash
 * is. base32 rather than base64 because the same encoding has to survive being
 * typed out as keystrokes later, where uppercase would need Shift and Caps
 * Lock would corrupt everything.
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include "nino_store.h"

#define LINK_NODE DT_NODELABEL(nino_cdc)

/* "DATA " plus the longest chunk the host may send, plus slack. */
#define LINE_MAX 320
#define RX_RING_SIZE 1024
#define CHUNK_BYTES_MAX 192

/* base32 is 8 characters per 5 bytes, rounded up. */
#define B32_CHARS_MAX (((CHUNK_BYTES_MAX * 8) + 4) / 5)

static const struct device *const link_dev = DEVICE_DT_GET(LINK_NODE);

RING_BUF_DECLARE(rx_ring, RX_RING_SIZE);
static K_SEM_DEFINE(rx_ready, 0, 1);

static char line[LINE_MAX];
static size_t line_len;
static bool line_overflow;
static uint8_t chunk[CHUNK_BYTES_MAX];
static char reply_buf[sizeof("DATA ") + B32_CHARS_MAX + 1];

/* ------------------------------------------------------------------ base32 -- */

/* RFC 4648 alphabet, lowercased. Returns the 5-bit value, or -1. */
static int b32_value(char c) {
    if (c >= 'a' && c <= 'z') {
        return c - 'a';
    }
    if (c >= 'A' && c <= 'Z') {
        return c - 'A'; /* tolerate uppercase; the encoding is case-blind */
    }
    if (c >= '2' && c <= '7') {
        return 26 + (c - '2');
    }
    return -1;
}

static const char b32_alphabet[] = "abcdefghijklmnopqrstuvwxyz234567";

/*
 * No '=' padding is emitted. The length is always known from context — the
 * host asked for exactly this many bytes — so padding would only be
 * characters to type later for no information.
 */
static size_t b32_encode(const uint8_t *src, size_t len, char *dst, size_t dst_max) {
    uint32_t acc = 0;
    int bits = 0;
    size_t out = 0;

    for (size_t i = 0; i < len; i++) {
        acc = (acc << 8) | src[i];
        bits += 8;

        while (bits >= 5) {
            bits -= 5;
            if (out + 1 >= dst_max) {
                return 0;
            }
            dst[out++] = b32_alphabet[(acc >> bits) & 0x1F];
        }
    }

    if (bits > 0) {
        if (out + 1 >= dst_max) {
            return 0;
        }
        dst[out++] = b32_alphabet[(acc << (5 - bits)) & 0x1F];
    }

    dst[out] = '\0';
    return out;
}

/* Returns bytes produced, or -1 on a bad character. Padding is ignored. */
static int b32_decode(const char *src, size_t src_len, uint8_t *dst, size_t dst_max) {
    uint32_t acc = 0;
    int bits = 0;
    size_t out = 0;

    for (size_t i = 0; i < src_len; i++) {
        if (src[i] == '=') {
            continue;
        }

        int v = b32_value(src[i]);
        if (v < 0) {
            return -1;
        }

        acc = (acc << 5) | (uint32_t)v;
        bits += 5;

        if (bits >= 8) {
            bits -= 8;
            if (out >= dst_max) {
                return -1;
            }
            dst[out++] = (uint8_t)(acc >> bits);
        }
    }

    return (int)out;
}

/* --------------------------------------------------------------------- io -- */

static void link_send(const char *text) {
    for (const char *p = text; *p != '\0'; p++) {
        uart_poll_out(link_dev, *p);
    }
    uart_poll_out(link_dev, '\n');
}

static void link_isr(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }

    while (uart_irq_rx_ready(dev)) {
        uint8_t buf[64];
        int n = uart_fifo_read(dev, buf, sizeof(buf));
        if (n <= 0) {
            break;
        }
        /* A full ring means the host outran us; dropping bytes corrupts the
         * blob, but the CRC at END is what catches that. */
        ring_buf_put(&rx_ring, buf, (uint32_t)n);
    }

    k_sem_give(&rx_ready);
}

/* ---------------------------------------------------------------- commands -- */

static void cmd_hello(void) {
    /* The CRC is part of the greeting because the reader needs it before it
     * has anything to check against, and there is no sense in a second
     * round trip for four bytes. */
    char reply[64];
    snprintf(reply, sizeof(reply), "NINO 1 %u %u %08x", (unsigned)nino_store_capacity(),
             (unsigned)nino_store_used(), (unsigned)nino_store_crc());
    link_send(reply);
}

static void cmd_begin(char *args) {
    char *total_str = strtok(args, " ");
    char *crc_str = strtok(NULL, " ");

    if (total_str == NULL || crc_str == NULL) {
        link_send("ERR");
        return;
    }

    uint32_t total = (uint32_t)strtoul(total_str, NULL, 10);
    uint32_t crc = (uint32_t)strtoul(crc_str, NULL, 16);

    int rc = nino_store_begin(total, crc);
    if (rc == -ENOSPC) {
        link_send("FULL");
    } else if (rc != 0) {
        link_send("ERR");
    } else {
        link_send("OK");
    }
}

static void cmd_data(char *args) {
    size_t len = strlen(args);

    int n = b32_decode(args, len, chunk, sizeof(chunk));
    if (n < 0) {
        link_send("BAD");
        return;
    }

    if (nino_store_append(chunk, (size_t)n) != 0) {
        link_send("BAD");
        return;
    }

    link_send("OK");
}

static void cmd_read(char *args) {
    char *offset_str = strtok(args, " ");
    char *len_str = strtok(NULL, " ");

    if (offset_str == NULL || len_str == NULL) {
        link_send("ERR");
        return;
    }

    size_t offset = (size_t)strtoul(offset_str, NULL, 10);
    size_t len = (size_t)strtoul(len_str, NULL, 10);

    if (len == 0 || len > CHUNK_BYTES_MAX) {
        link_send("ERR");
        return;
    }

    if (nino_store_read(offset, chunk, len) != 0) {
        link_send("ERR");
        return;
    }

    memcpy(reply_buf, "DATA ", 5);
    if (b32_encode(chunk, len, reply_buf + 5, sizeof(reply_buf) - 5) == 0) {
        link_send("ERR");
        return;
    }

    link_send(reply_buf);
}

static void cmd_end(void) {
    uint32_t actual = 0;
    int rc = nino_store_commit(&actual);

    if (rc == 0) {
        link_send("OK");
        return;
    }

    /* Report what was promised against what arrived. nino_store_crc() would be
     * wrong here: after a failed commit there is no committed blob, so it
     * reads back as zero. */
    char reply[64];
    snprintf(reply, sizeof(reply), "BAD %08x %08x", (unsigned)nino_store_expected_crc(),
             (unsigned)actual);
    link_send(reply);
}

static void handle_line(char *text) {
    while (*text == ' ') {
        text++;
    }
    if (*text == '\0') {
        return;
    }

    char *args = strchr(text, ' ');
    if (args != NULL) {
        *args++ = '\0';
    } else {
        args = text + strlen(text);
    }

    if (strcmp(text, "HELLO") == 0) {
        cmd_hello();
    } else if (strcmp(text, "ERASE") == 0) {
        link_send(nino_store_erase() == 0 ? "OK" : "ERR");
    } else if (strcmp(text, "BEGIN") == 0) {
        cmd_begin(args);
    } else if (strcmp(text, "DATA") == 0) {
        cmd_data(args);
    } else if (strcmp(text, "END") == 0) {
        cmd_end();
    } else if (strcmp(text, "READ") == 0) {
        cmd_read(args);
    } else {
        link_send("ERR");
    }
}

static void link_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    if (!device_is_ready(link_dev)) {
        LOG_ERR("NINO link: CDC ACM not ready");
        return;
    }

    uart_irq_callback_user_data_set(link_dev, link_isr, NULL);
    uart_irq_rx_enable(link_dev);

    while (true) {
        k_sem_take(&rx_ready, K_FOREVER);

        uint8_t byte;
        while (ring_buf_get(&rx_ring, &byte, 1) == 1) {
            if (byte == '\r') {
                continue;
            }

            if (byte == '\n') {
                /*
                 * Exactly one reply per line, always. An overlong line is
                 * refused rather than dropped: dropping it silently leaves the
                 * host blocked on a read that will never complete, which looks
                 * like a hung keyboard rather than a rejected command.
                 */
                if (line_overflow) {
                    link_send("BAD");
                    line_overflow = false;
                } else {
                    line[line_len] = '\0';
                    handle_line(line);
                }
                line_len = 0;
                continue;
            }

            if (line_len < sizeof(line) - 1) {
                line[line_len++] = (char)byte;
            } else {
                /* Keep consuming to the newline, but never truncate into a
                 * command that happens to still parse. */
                line_overflow = true;
            }
        }
    }
}

K_THREAD_DEFINE(nino_link_tid, CONFIG_NINO_LINK_STACK_SIZE, link_thread, NULL, NULL, NULL,
                CONFIG_NINO_LINK_PRIORITY, 0, 0);
