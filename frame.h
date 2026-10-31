#pragma once
// frame.h - Framed data link: preamble, sync word, sequence number, length, payload, CRC-16.
// Pure C++ (no Arduino code) so it can be unit-tested on a PC.
//
// Frame on the wire (bytes, each sent MSB first):
//   preamble 0x55 x4 | sync 0x2D 0xD4 | seq (2) | len (1) | payload (len) | CRC-16 (2)
// The CRC covers seq, len and payload.
#include <stdint.h>
#include <string.h>

namespace frm {

const uint16_t SYNC = 0x2DD4;            // sync word the receiver hunts for
const uint8_t  PREAMBLE = 0x55;          // 01010101: keeps the decision threshold trained
const int PREAMBLE_BYTES = 4;
const int MAX_PAYLOAD = 32;
const int HEADER_BYTES = 3;              // seq (2) + len (1)
const int MAX_FRAME_BYTES = PREAMBLE_BYTES + 2 + HEADER_BYTES + MAX_PAYLOAD + 2;

// CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF ("123456789" -> 0x29B1)
inline uint16_t crc16(const uint8_t *d, int n) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < n; i++) {
    crc ^= (uint16_t)(d[i] << 8);
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// Builds a whole frame into 'out' and returns its length in bytes (0 if the payload is too long)
inline int build(uint8_t *out, uint16_t seq, const char *payload, int len) {
  if (len < 0 || len > MAX_PAYLOAD) return 0;
  int n = 0;
  for (int i = 0; i < PREAMBLE_BYTES; i++) out[n++] = PREAMBLE;
  out[n++] = SYNC >> 8;
  out[n++] = SYNC & 0xFF;
  int body = n;
  out[n++] = seq >> 8;
  out[n++] = seq & 0xFF;
  out[n++] = (uint8_t)len;
  memcpy(out + n, payload, len);
  n += len;
  uint16_t crc = crc16(out + body, HEADER_BYTES + len);
  out[n++] = crc >> 8;
  out[n++] = crc & 0xFF;
  return n;
}

// Bit-serial receiver: hunts for the sync word, collects the frame, checks the CRC
struct Receiver {
  enum Result { NONE = 0, GOOD = 1, BAD = -1 };

  uint32_t good = 0, crcErrors = 0, lengthErrors = 0, missed = 0;
  uint16_t lastSeq = 0;
  bool haveSeq = false;
  uint32_t badSinceGood = 0;             // frames already counted as bad since the last good one
  char lastText[MAX_PAYLOAD + 1] = "";

  bool hunting = true;
  uint16_t shift = 0;
  uint8_t buf[HEADER_BYTES + MAX_PAYLOAD + 2] = {};
  int nbits = 0, needBytes = 0;

  void resync() { hunting = true; shift = 0; }

  void resetAll() { *this = Receiver(); }

  // Feed one decided bit. Returns GOOD or BAD when a frame ends, otherwise NONE.
  Result push(uint8_t bit) {
    bit &= 1;
    if (hunting) {
      shift = (uint16_t)((shift << 1) | bit);
      if (shift == SYNC) {
        hunting = false;
        nbits = 0;
        needBytes = HEADER_BYTES;
        memset(buf, 0, sizeof(buf));
      }
      return NONE;
    }

    buf[nbits >> 3] = (uint8_t)((buf[nbits >> 3] << 1) | bit);
    nbits++;
    if (nbits < needBytes * 8) return NONE;

    if (needBytes == HEADER_BYTES) {                 // header complete: now we know the length
      int len = buf[2];
      if (len > MAX_PAYLOAD) { lengthErrors++; badSinceGood++; resync(); return BAD; }
      needBytes = HEADER_BYTES + len + 2;
      return NONE;
    }

    int len = buf[2];
    uint16_t got = (uint16_t)((buf[HEADER_BYTES + len] << 8) | buf[HEADER_BYTES + len + 1]);
    Result r;
    if (crc16(buf, HEADER_BYTES + len) == got) {
      uint16_t seq = (uint16_t)((buf[0] << 8) | buf[1]);
      // Frames absent between two good ones; those already counted as bad are not counted again
      if (haveSeq) {
        uint32_t gap = (uint16_t)(seq - lastSeq - 1);
        missed += (gap > badSinceGood) ? gap - badSinceGood : 0;
      }
      badSinceGood = 0;
      haveSeq = true;
      lastSeq = seq;
      memcpy(lastText, buf + HEADER_BYTES, len);
      lastText[len] = '\0';
      good++;
      r = GOOD;
    } else {
      crcErrors++;
      badSinceGood++;
      r = BAD;
    }
    resync();
    return r;
  }
};

}  // namespace frm