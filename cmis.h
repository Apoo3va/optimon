#pragma once
// cmis.h - Register map and decoders for CMIS 5.x modules (QSFP-DD, OSFP, ...).
// Pure C++ (no Arduino code) so it can be unit-tested on a PC.
// Offsets follow OIF CMIS 5.x. Upper memory is paged: byte 126 selects the bank, byte 127 the page.
#include <stdint.h>
#include "sff8472.h"     // same units for monitors and thresholds, same Limits / decode helpers

namespace cmis {

const uint8_t ADDR = 0x50;

// SFF-8024 identifiers that mean "this module uses CMIS"
const uint8_t ID_QSFP_DD = 0x18, ID_OSFP = 0x19, ID_SFP_DD = 0x1A, ID_DSFP = 0x1B, ID_QSFP_CMIS = 0x1E;
inline bool isCmisId(uint8_t id) {
  return id == ID_QSFP_DD || id == ID_OSFP || id == ID_SFP_DD || id == ID_DSFP || id == ID_QSFP_CMIS;
}
const uint8_t ID_SFP = 0x03;             // SFF-8472 module

// Lower memory (always visible, bytes 0-127)
const uint8_t REV = 1, MEM_MODEL = 2, STATE = 3, MOD_FLAGS = 8, TEMPVCC_FLAGS = 9;
const uint8_t TEMP = 14, VCC = 16, MOD_CTRL = 26, FW_MAJOR = 39, FW_MINOR = 40, MEDIA_TYPE = 85;
const uint8_t BANK_SEL = 126, PAGE_SEL = 127;
const uint8_t CTRL_LOWPWR_SW = 0x10, CTRL_SW_RESET = 0x08;
const uint8_t FLAG_STATE_CHANGED = 0x01;
const uint8_t MEM_FLAT = 0x80;           // byte 2 bit 7: 1 = flat memory (no pages)

// Upper pages
const uint8_t PAGE_ADMIN = 0x00, PAGE_THRESHOLDS = 0x02, PAGE_LANE_CTRL = 0x10, PAGE_LANE_STATUS = 0x11;
const uint8_t P00_VENDOR_NAME = 129, P00_VENDOR_PN = 148, P00_VENDOR_SN = 166, P00_CHECKSUM = 222;
const uint8_t P02_TEMP = 128, P02_VCC = 136, P02_TXPWR = 176, P02_TXBIAS = 184, P02_RXPWR = 192;
const uint8_t P10_DP_DEINIT = 128, P10_TX_DISABLE = 130;
const uint8_t P11_DP_STATE = 128, P11_DP_CHANGED = 134, P11_TXPWR_HA = 140, P11_RX_LOS = 148;
const uint8_t P11_RXPWR_HA = 150, P11_TXPWR = 154, P11_TXBIAS = 170, P11_RXPWR = 186;

enum ModuleState { MS_UNKNOWN = 0, MS_LOWPWR = 1, MS_PWRUP = 2, MS_READY = 3, MS_PWRDN = 4, MS_FAULT = 5 };
enum DataPathState { DP_UNKNOWN = 0, DP_DEACTIVATED = 1, DP_INIT = 2, DP_DEINIT = 3, DP_ACTIVATED = 4,
                     DP_TXTURNON = 5, DP_TXTURNOFF = 6, DP_INITIALIZED = 7 };

inline ModuleState moduleState(uint8_t byte3) {
  uint8_t s = (byte3 >> 1) & 0x07;
  return (s >= MS_LOWPWR && s <= MS_FAULT) ? (ModuleState)s : MS_UNKNOWN;
}
inline bool intlAsserted(uint8_t byte3) { return (byte3 & 0x01) == 0; }   // bit 0 = 1 means "not asserted"

// Data path state of lane 1..8 from page 11h bytes 128-131 (odd lane = low nibble)
inline DataPathState dpState(const uint8_t *p11_128, int lane) {
  uint8_t b = p11_128[(lane - 1) / 2];
  uint8_t s = (lane % 2) ? (b & 0x0F) : (b >> 4);
  return (s >= DP_DEACTIVATED && s <= DP_INITIALIZED) ? (DataPathState)s : DP_UNKNOWN;
}

inline const char *moduleStateName(ModuleState s) {
  switch (s) {
    case MS_LOWPWR: return "ModuleLowPwr";
    case MS_PWRUP:  return "ModulePwrUp";
    case MS_READY:  return "ModuleReady";
    case MS_PWRDN:  return "ModulePwrDn";
    case MS_FAULT:  return "ModuleFault";
    default:        return "Unknown";
  }
}
inline const char *moduleStateShort(ModuleState s) {
  switch (s) {
    case MS_LOWPWR: return "LPWR";
    case MS_PWRUP:  return "PWUP";
    case MS_READY:  return "RDY";
    case MS_PWRDN:  return "PWDN";
    case MS_FAULT:  return "FLT";
    default:        return "?";
  }
}
inline const char *dpStateName(DataPathState s) {
  switch (s) {
    case DP_DEACTIVATED: return "DPDeactivated";
    case DP_INIT:        return "DPInit";
    case DP_DEINIT:      return "DPDeinit";
    case DP_ACTIVATED:   return "DPActivated";
    case DP_TXTURNON:    return "DPTxTurnOn";
    case DP_TXTURNOFF:   return "DPTxTurnOff";
    case DP_INITIALIZED: return "DPInitialized";
    default:             return "Unknown";
  }
}

// Thresholds from page 02h (bytes 128-255 of the page, so p02[0] is byte 128).
// Each parameter: high alarm, low alarm, high warning, low warning, same units as SFF-8472.
// Note the order differs from SFF-8472: Tx power comes before Tx bias.
inline sff::Limits limitsFromPage02(const uint8_t *p02_128, sff::Param p) {
  uint8_t off;
  switch (p) {
    case sff::P_TEMP: off = P02_TEMP;   break;
    case sff::P_VCC:  off = P02_VCC;    break;
    case sff::P_BIAS: off = P02_TXBIAS; break;
    case sff::P_TX:   off = P02_TXPWR;  break;
    default:          off = P02_RXPWR;  break;
  }
  const uint8_t *b = p02_128 + (off - 128);
  return { sff::decode(p, sff::be16(b)), sff::decode(p, sff::be16(b + 2)),
           sff::decode(p, sff::be16(b + 4)), sff::decode(p, sff::be16(b + 6)) };
}

}  // namespace cmis
