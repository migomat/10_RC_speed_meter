// CRC-32 IEEE 802.3 (jak zlib.crc32 / binascii.crc32): poly 0xEDB88320,
// init 0xFFFFFFFF, xorout 0xFFFFFFFF.
#pragma once

#include <stdint.h>
#include <stddef.h>

static inline uint32_t crc32_begin(void) { return 0xFFFFFFFFu; }
uint32_t crc32_update(uint32_t crc, const void* data, size_t len);
static inline uint32_t crc32_end(uint32_t crc) { return crc ^ 0xFFFFFFFFu; }
static inline uint32_t crc32(const void* data, size_t len) {
  return crc32_end(crc32_update(crc32_begin(), data, len));
}
