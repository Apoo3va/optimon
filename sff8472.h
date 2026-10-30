#pragma once
// sff8472.h - Decoder for SFF-8472 SFP diagnostics.
// Pure C++ (no Arduino code) so it can be unit-tested on a PC.
#include <stdint.h>
#include <string.h>
#include <math.h>

namespace sff {

const uint8_t ADDR_A0 = 0x50;            // serial ID page
const uint8_t ADDR_A2 = 0x51;            // diagnostics page

// A0h offsets
const uint8_t A0_VENDOR_NAME = 20;       // 16 ASCII bytes
const uint8_t A0_VENDOR_PN   = 40;       // 16 ASCII bytes
const uint8_t A0_WAVELENGTH  = 60;       // 2 bytes, nm
const uint8_t A0_CC_BASE     = 63;       // checksum of bytes 0..62
const uint8_t A0_DIAG_TYPE   = 92;
const uint8_t DIAG_IMPLEMENTED = 0x40;

// A2h offsets
const uint8_t A2_THRESHOLDS = 0;         // 5 params x 4 limits x 2 bytes = 40 bytes
const uint8_t A2_CC_DMI     = 95;        // checksum of bytes 0..94
const uint8_t A2_MONITORS   = 96;        // 5 params x 2 bytes = 10 bytes
const uint8_t A2_STATUS     = 110;
const uint8_t STATUS_TX_DISABLE = 0x80;
const uint8_t STATUS_RX_LOS     = 0x02;

enum Param { P_TEMP = 0, P_VCC, P_BIAS, P_TX, P_RX, NUM_PARAMS };
enum Level { L_OK = 0, L_WARN_LOW, L_WARN_HIGH, L_ALARM_LOW, L_ALARM_HIGH };  // ordered by severity

struct Limits { float highAlarm, lowAlarm, highWarn, lowWarn; };

inline uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }

inline bool checksumOk(const uint8_t *m, int from, int to, uint8_t expected) {
  uint32_t s = 0;
  for (int i = from; i <= to; i++) s += m[i];
  return (uint8_t)(s & 0xFF) == expected;
}

// Optical power: raw is in units of 0.1 uW, so mW = raw * 1e-4
inline float toDbm(uint16_t raw) { return raw == 0 ? -40.0f : 10.0f * log10f(raw * 1e-4f); }

inline float decode(Param p, uint16_t raw) {
  switch (p) {
    case P_TEMP: return (int16_t)raw / 256.0f;   // signed, 1/256 degC
    case P_VCC:  return raw * 100e-6f;           // 100 uV per LSB
    case P_BIAS: return raw * 2e-3f;             // 2 uA per LSB -> mA
    case P_TX:
    case P_RX:   return toDbm(raw);
    default:     return 0;
  }
}

inline float decodeMonitor(const uint8_t *mon, Param p) { return decode(p, be16(mon + 2 * p)); }

inline Limits decodeLimits(const uint8_t *thr, Param p) {
  const uint8_t *b = thr + 8 * p;
  return { decode(p, be16(b)), decode(p, be16(b + 2)), decode(p, be16(b + 4)), decode(p, be16(b + 6)) };
}

inline Level classify(float v, const Limits &l) {
  if (v > l.highAlarm) return L_ALARM_HIGH;
  if (v < l.lowAlarm)  return L_ALARM_LOW;
  if (v > l.highWarn)  return L_WARN_HIGH;
  if (v < l.lowWarn)   return L_WARN_LOW;
  return L_OK;
}

inline bool isAlarm(Level l) { return l == L_ALARM_LOW || l == L_ALARM_HIGH; }

inline const char *levelName(Level l) {
  switch (l) {
    case L_OK:         return "OK";
    case L_WARN_LOW:   return "LO-W";
    case L_WARN_HIGH:  return "HI-W";
    case L_ALARM_LOW:  return "LO-A";
    case L_ALARM_HIGH: return "HI-A";
  }
  return "?";
}

// Copy a space-padded ASCII field and trim trailing spaces
inline void asciiField(char *out, const uint8_t *in, int len) {
  memcpy(out, in, len);
  out[len] = '\0';
  for (int i = len - 1; i >= 0 && out[i] == ' '; i--) out[i] = '\0';
}

}  // namespace sff