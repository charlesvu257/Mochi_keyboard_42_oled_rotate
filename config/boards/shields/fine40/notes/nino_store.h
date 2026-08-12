/*
 * NINO — flash-backed note store
 *
 * Owns nino_partition outright. The store holds one opaque blob: the firmware
 * never parses it, only counts it, hands it back, and checks it survived.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>

/* Bytes available for the blob itself, once the header is accounted for. */
size_t nino_store_capacity(void);

/* Bytes currently stored. Zero when the store is empty or was never committed. */
size_t nino_store_used(void);

/* CRC32 of the stored blob, as recorded at commit. Meaningless when empty. */
uint32_t nino_store_crc(void);

/* What the sender said the CRC would be, for reporting a mismatch back. */
uint32_t nino_store_expected_crc(void);

/* Erase everything. */
int nino_store_erase(void);

/*
 * Start receiving a blob. Erases the partition and remembers what the sender
 * says is coming, so commit can check it arrived intact.
 * Returns -ENOSPC if total exceeds capacity.
 */
int nino_store_begin(uint32_t total, uint32_t expected_crc);

/* Append to the blob in progress. */
int nino_store_append(const uint8_t *data, size_t len);

/*
 * Finish. Writes the header only if the length and CRC both match what begin
 * declared, so a half-finished transfer leaves the store readably empty rather
 * than subtly wrong. Passes the CRC actually received back through actual_crc.
 */
int nino_store_commit(uint32_t *actual_crc);

/* Read committed bytes. Offsets are relative to the blob, not the partition. */
int nino_store_read(size_t offset, uint8_t *buf, size_t len);
