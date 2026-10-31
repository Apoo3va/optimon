// sfp.chip.c - Simulated optical transceiver + fiber loopback link + optical receiver, for Wokwi.
//
// Two module types, chosen with the "Module type" slider (moving it is a hot-swap):
//   0 = SFP with SFF-8472 diagnostics: I2C 0x50 (A0h) + 0x51 (A2h), LOS pin active high
//   1 = QSFP-DD with CMIS 5.x:         I2C 0x50 only, paged memory, module state machine,
//                                      data path state, latched flags, IntL pin active low
// Both share the same laser, fiber and receiver physics.
#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define ADDR_A0 0x50            // SFP serial ID page ("A0h"); also the CMIS address
#define ADDR_A2 0x51            // SFP diagnostics page ("A2h")

// ---- Optical link constants (1550 nm, standard single-mode fiber) ----
#define FIBER_LOSS_DB_PER_KM  0.20f
#define CONNECTOR_LOSS_DB     0.50f   // per connector, 2 in the link
#define SPLICE_LOSS_DB        0.10f   // per splice, 2 in the link
#define DARK_DBM            (-40.0f)  // "no light" floor
#define LOS_ASSERT_DBM      (-30.0f)  // LOS turns on at or below this
#define LOS_DEASSERT_DBM    (-28.0f)  // LOS turns off only above this (2 dB hysteresis)

// ---- Optical receiver model: photodiode + transimpedance amplifier ----
#define RX_TICK_NS          50000     // receiver output updated every 50 us
#define RX_TICK_US          50.0f
#define RX_OFFSET_V         0.10f     // output with no light
#define RX_GAIN_V_PER_MW    16.0f     // responsivity x amplifier gain
#define RX_RAIL_V           3.00f     // amplifier saturates here (overload)

// ---- CMIS (OIF CMIS 5.x) register map, lower memory ----
#define CMIS_ID_QSFP_DD       0x18    // SFF-8024 identifier
#define CMIS_REV_5_2          0x52
#define CMIS_STATE            3       // bits 3:1 module state, bit 0 = 1 when IntL is NOT asserted
#define CMIS_MOD_FLAGS        8       // latched: bit 0 ModuleStateChanged
#define CMIS_TEMPVCC_FLAGS    9       // latched: temp HA/LA/HW/LW bits 0-3, Vcc HA/LA/HW/LW bits 4-7
#define CMIS_TEMP             14      // S16, 1/256 degC
#define CMIS_VCC              16      // U16, 100 uV
#define CMIS_MOD_CTRL         26      // bit 4 LowPwrRequestSW, bit 3 SoftwareReset
#define CMIS_FW_MAJOR         39
#define CMIS_FW_MINOR         40
#define CMIS_MEDIA_TYPE       85      // 0x02 = single-mode fiber
#define CMIS_APP_DESC1        86
#define CMIS_BANK_SEL         126
#define CMIS_PAGE_SEL         127
#define CTRL_LOWPWR_SW        0x10
#define CTRL_SW_RESET         0x08

enum { ST_LOWPWR = 1, ST_PWRUP = 2, ST_READY = 3, ST_PWRDN = 4, ST_FAULT = 5 };   // module states
enum { DP_DEACTIVATED = 1, DP_INIT = 2, DP_DEINIT = 3, DP_ACTIVATED = 4 };       // data path states

// Upper pages (offsets are 128-255 within the page)
#define P00_ID          128
#define P00_VENDOR_NAME 129
#define P00_VENDOR_PN   148
#define P00_VENDOR_REV  164
#define P00_VENDOR_SN   166
#define P00_DATE        182
#define P00_POWER_CLASS 200
#define P00_MAX_POWER   201
#define P00_CHECKSUM    222           // sum of bytes 128..221
#define P02_TEMP_THR    128           // each: high alarm, low alarm, high warning, low warning
#define P02_VCC_THR     136
#define P02_TXPWR_THR   176
#define P02_TXBIAS_THR  184
#define P02_RXPWR_THR   192
#define P10_DP_DEINIT   128           // bit n = 1: hold data path of lane n+1 deactivated
#define P10_TX_DISABLE  130           // bit n = 1: lane n+1 Tx output disabled
#define P11_DP_STATE    128           // 4 bits per lane, lane 1 in the low nibble of byte 128
#define P11_DP_CHANGED  134           // latched
#define P11_TXPWR_HA    140           // latched Tx power / bias flags 140-147
#define P11_RX_LOS      148           // latched Rx LOS
#define P11_RXPWR_HA    150           // latched Rx power flags 150-153 (HA, LA, HW, LW)
#define P11_TXPWR       154           // U16, 0.1 uW, lane 1
#define P11_TXBIAS      170           // U16, 2 uA, lane 1
#define P11_RXPWR       186           // U16, 0.1 uW, lane 1

// Simulated timing of the module (in 10 ms ticks)
#define PWRUP_TICKS     200           // 2.0 s: DSP boot, laser warm-up
#define PWRDN_TICKS     50            // 0.5 s
#define DP_INIT_TICKS   100           // 1.0 s: laser tuning, data path lock

typedef struct {
  int type;                           // 0 = SFP, 1 = CMIS
  // SFP memory
  uint8_t a0[256];
  uint8_t a2[256];
  uint8_t live[10];                   // newest SFP diagnostics, latched into a2 when a read starts
  uint8_t live_status;
  // CMIS memory: lower page + upper pages 00h, 02h, 10h, 11h (bank 0)
  uint8_t lower[128];
  uint8_t p00[256], p02[256], p10[256], p11[256];
  int mod_state, mod_timer, dp_state, dp_timer;
  // I2C transaction state
  uint8_t *page;                      // SFP page addressed by the current transaction
  bool page_is_a2;
  bool cmis_txn;
  uint8_t ptr;                        // address pointer (wraps at 256)
  bool expect_offset;
  // Physics
  bool los;                           // current LOS state (with hysteresis)
  float p1_mw, p0_mw;                 // received optical power for a '1' and a '0'
  float v_filt;                       // receiver output before noise (bandwidth-limited)
  uint32_t attr_temp, attr_tx, attr_length, attr_bend, attr_gain, attr_aging, attr_cut;
  uint32_t attr_er, attr_tau, attr_noise, attr_type;
  pin_t pin_los, pin_txdis, pin_txdata, pin_rxout;
  timer_t timer, rx_timer;
} chip_state_t;

// ---------- Encoding helpers (SFF-8472 and CMIS use the same units) ----------
static void put16(uint8_t *m, int off, uint16_t v) { m[off] = v >> 8; m[off + 1] = v & 0xFF; }

static uint16_t clamp_u16(float v) {
  if (v < 0) return 0;
  if (v > 65535) return 65535;
  return (uint16_t)roundf(v);
}
static uint16_t enc_temp(float c) {                 // signed, 1/256 degC per LSB
  float r = roundf(c * 256.0f);
  if (r > 32767) r = 32767;
  if (r < -32768) r = -32768;
  return (uint16_t)(int16_t)r;
}
static uint16_t enc_vcc(float v)   { return clamp_u16(v / 100e-6f); }                       // 100 uV per LSB
static uint16_t enc_bias(float mA) { return clamp_u16(mA / 2e-3f); }                        // 2 uA per LSB
static uint16_t enc_dbm(float dbm) { return clamp_u16(powf(10.0f, dbm / 10.0f) * 1e4f); }  // 0.1 uW per LSB

static uint8_t checksum(const uint8_t *m, int from, int to) {
  uint32_t s = 0;
  for (int i = from; i <= to; i++) s += m[i];
  return (uint8_t)(s & 0xFF);
}
static void put_ascii(uint8_t *m, int off, int len, const char *s) {
  memset(m + off, ' ', len);
  int n = (int)strlen(s);
  memcpy(m + off, s, n < len ? n : len);
}
static void put_limits(uint8_t *m, int off, uint16_t ha, uint16_t la, uint16_t hw, uint16_t lw) {
  put16(m, off, ha); put16(m, off + 2, la); put16(m, off + 4, hw); put16(m, off + 6, lw);
}

// Alarm and warning limits, shared by both module types
#define TEMP_LIMITS   75.0f, -5.0f, 70.0f, 0.0f
#define VCC_LIMITS    3.6f, 3.0f, 3.5f, 3.1f
#define BIAS_LIMITS   80.0f, 2.0f, 70.0f, 4.0f
#define TXPWR_LIMITS  4.0f, -8.0f, 3.0f, -7.0f
#define RXPWR_LIMITS  0.0f, -24.0f, -1.0f, -22.0f

// Flag bits in the order the four thresholds are stored: high alarm, low alarm, high warning, low warning
static uint8_t limit_flags(float v, float ha, float la, float hw, float lw) {
  return (v > ha ? 1 : 0) | (v < la ? 2 : 0) | (v > hw ? 4 : 0) | (v < lw ? 8 : 0);
}

// ---------- SFP memory map ----------
static void init_a0(uint8_t *a0) {
  memset(a0, 0, 256);
  a0[0] = 0x03;                          // identifier: SFP/SFP+
  a0[1] = 0x04;                          // serial ID over 2-wire interface
  a0[2] = 0x07;                          // connector: LC
  put_ascii(a0, 20, 16, "APOORVA OPTICS");
  put_ascii(a0, 40, 16, "SFP-SIM-1550");
  put_ascii(a0, 56, 4, "1.0");
  put16(a0, 60, 1550);                   // laser wavelength in nm
  a0[63] = checksum(a0, 0, 62);          // CC_BASE
  put_ascii(a0, 68, 16, "OPT0000000001");
  put_ascii(a0, 84, 8, "261028");
  a0[92] = 0x68;                         // DDM implemented, internally calibrated, avg Rx power
  a0[95] = checksum(a0, 64, 94);         // CC_EXT
}

static void init_a2(uint8_t *a2) {
  memset(a2, 0, 256);
  //            offset  high alarm        low alarm         high warning      low warning
  put_limits(a2, 0,  enc_temp(75),     enc_temp(-5),     enc_temp(70),     enc_temp(0));      // temperature
  put_limits(a2, 8,  enc_vcc(3.6f),    enc_vcc(3.0f),    enc_vcc(3.5f),    enc_vcc(3.1f));    // supply voltage
  put_limits(a2, 16, enc_bias(80),     enc_bias(2),      enc_bias(70),     enc_bias(4));      // laser bias
  put_limits(a2, 24, enc_dbm(4),       enc_dbm(-8),      enc_dbm(3),       enc_dbm(-7));      // Tx power
  put_limits(a2, 32, enc_dbm(0),       enc_dbm(-24),     enc_dbm(-1),      enc_dbm(-22));     // Rx power
  a2[95] = checksum(a2, 0, 94);          // CC_DMI
}

// ---------- CMIS memory map ----------
static void init_cmis(chip_state_t *chip) {
  uint8_t *lo = chip->lower;
  memset(lo, 0, 128);
  memset(chip->p00, 0, 256); memset(chip->p02, 0, 256);
  memset(chip->p10, 0, 256); memset(chip->p11, 0, 256);

  lo[0] = CMIS_ID_QSFP_DD;
  lo[1] = CMIS_REV_5_2;
  lo[2] = 0x00;                          // paged memory, 400 kHz I2C
  lo[CMIS_MOD_CTRL] = CTRL_LOWPWR_SW;    // simplification: power up held in low power until the host releases it
  lo[CMIS_FW_MAJOR] = 1;
  lo[CMIS_FW_MINOR] = 0;
  lo[CMIS_MEDIA_TYPE] = 0x02;            // single-mode fiber
  lo[CMIS_APP_DESC1] = 0xFF;             // no application descriptors (end of list)

  uint8_t *p0 = chip->p00;
  p0[P00_ID] = CMIS_ID_QSFP_DD;
  put_ascii(p0, P00_VENDOR_NAME, 16, "APOORVA OPTICS");
  put_ascii(p0, P00_VENDOR_PN, 16, "QDD-SIM-ZR");
  put_ascii(p0, P00_VENDOR_REV, 2, "A0");
  put_ascii(p0, P00_VENDOR_SN, 16, "OPT0000000002");
  put_ascii(p0, P00_DATE, 8, "261031");
  p0[P00_POWER_CLASS] = 0xE0;            // power class 8 (> 10 W)
  p0[P00_MAX_POWER] = 80;                // 80 x 0.25 W = 20 W, typical for a coherent module
  p0[P00_CHECKSUM] = checksum(p0, 128, 221);

  uint8_t *p2 = chip->p02;
  put_limits(p2, P02_TEMP_THR,   enc_temp(75), enc_temp(-5), enc_temp(70), enc_temp(0));
  put_limits(p2, P02_VCC_THR,    enc_vcc(3.6f), enc_vcc(3.0f), enc_vcc(3.5f), enc_vcc(3.1f));
  put_limits(p2, P02_TXPWR_THR,  enc_dbm(4), enc_dbm(-8), enc_dbm(3), enc_dbm(-7));
  put_limits(p2, P02_TXBIAS_THR, enc_bias(80), enc_bias(2), enc_bias(70), enc_bias(4));
  put_limits(p2, P02_RXPWR_THR,  enc_dbm(0), enc_dbm(-24), enc_dbm(-1), enc_dbm(-22));

  chip->p10[P10_DP_DEINIT] = 0x01;       // lane 1 held deactivated until the host releases it

  chip->mod_state = ST_LOWPWR;
  chip->mod_timer = 0;
  chip->dp_state = DP_DEACTIVATED;
  chip->dp_timer = 0;
}

static uint8_t *cmis_upper_page(chip_state_t *chip) {
  if (chip->lower[CMIS_BANK_SEL] != 0) return NULL;   // only bank 0 is implemented
  switch (chip->lower[CMIS_PAGE_SEL]) {
    case 0x00: return chip->p00;
    case 0x02: return chip->p02;
    case 0x10: return chip->p10;
    case 0x11: return chip->p11;
    default:   return NULL;                           // unimplemented page reads as zeros
  }
}

static bool cmis_any_flag(chip_state_t *chip) {
  for (int i = CMIS_MOD_FLAGS; i <= 13; i++) if (chip->lower[i]) return true;
  for (int i = P11_DP_CHANGED; i <= 153; i++) if (chip->p11[i]) return true;
  return false;
}

static void cmis_set_mod_state(chip_state_t *chip, int s) {
  if (chip->mod_state != s) chip->lower[CMIS_MOD_FLAGS] |= 0x01;   // L-ModuleStateChanged
  chip->mod_state = s;
  chip->mod_timer = 0;
}

static void cmis_set_dp_state(chip_state_t *chip, int s) {
  if (chip->dp_state != s) chip->p11[P11_DP_CHANGED] |= 0x01;      // L-DPStateChanged, lane 1
  chip->dp_state = s;
  chip->dp_timer = 0;
}

// Module and data path state machines, one 10 ms tick
static void cmis_step(chip_state_t *chip) {
  uint8_t *lo = chip->lower;
  if (lo[CMIS_MOD_CTRL] & CTRL_SW_RESET) { init_cmis(chip); return; }
  bool lowpwr_req = lo[CMIS_MOD_CTRL] & CTRL_LOWPWR_SW;

  switch (chip->mod_state) {
    case ST_LOWPWR:
      if (!lowpwr_req) cmis_set_mod_state(chip, ST_PWRUP);
      break;
    case ST_PWRUP:
      if (lowpwr_req) cmis_set_mod_state(chip, ST_PWRDN);
      else if (++chip->mod_timer >= PWRUP_TICKS) cmis_set_mod_state(chip, ST_READY);
      break;
    case ST_READY:
      if (lowpwr_req) cmis_set_mod_state(chip, ST_PWRDN);
      break;
    case ST_PWRDN:
      if (++chip->mod_timer >= PWRDN_TICKS) cmis_set_mod_state(chip, ST_LOWPWR);
      break;
  }

  bool deinit = chip->p10[P10_DP_DEINIT] & 0x01;
  if (chip->mod_state != ST_READY) {
    if (chip->dp_state != DP_DEACTIVATED) cmis_set_dp_state(chip, DP_DEACTIVATED);
  } else {
    switch (chip->dp_state) {
      case DP_DEACTIVATED:
        if (!deinit) cmis_set_dp_state(chip, DP_INIT);
        break;
      case DP_INIT:
        if (deinit) cmis_set_dp_state(chip, DP_DEACTIVATED);
        else if (++chip->dp_timer >= DP_INIT_TICKS) cmis_set_dp_state(chip, DP_ACTIVATED);
        break;
      case DP_ACTIVATED:
        if (deinit) cmis_set_dp_state(chip, DP_DEACTIVATED);
        break;
    }
  }
  // Lane 1 in the low nibble of byte 128; lanes 2-8 not used (deactivated)
  chip->p11[P11_DP_STATE] = (uint8_t)((DP_DEACTIVATED << 4) | chip->dp_state);
  chip->p11[129] = chip->p11[130] = chip->p11[131] = (DP_DEACTIVATED << 4) | DP_DEACTIVATED;
}

// Small measurement noise on the diagnostics, +/- 0.05 dB
static float noise_db(void) { return ((rand() % 21) - 10) * 0.005f; }

// Gaussian random number (mean 0, std dev 1), Box-Muller method
static float gauss(void) {
  float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
  float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
  return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

static void set_type(chip_state_t *chip, int type) {
  chip->type = type;
  init_a0(chip->a0);
  init_a2(chip->a2);
  init_cmis(chip);
  chip->page = NULL;
  printf("Module inserted: %s\n", type ? "QSFP-DD, CMIS 5.2 (I2C 0x50, IntL active low)"
                                       : "SFP, SFF-8472 (I2C 0x50 + 0x51, LOS active high)");
}

// ---------- Runs every 10 ms: module logic + laser + fiber physics + diagnostics ----------
static void update_monitors(void *user_data) {
  chip_state_t *chip = user_data;
  int type = attr_read_float(chip->attr_type) >= 0.5f ? 1 : 0;
  if (type != chip->type) set_type(chip, type);          // slider moved: hot-swap
  if (chip->type == 1) cmis_step(chip);

  float temp     = attr_read_float(chip->attr_temp);
  float tx_set   = attr_read_float(chip->attr_tx);
  float length   = attr_read_float(chip->attr_length);
  float bend     = attr_read_float(chip->attr_bend);
  float gain     = attr_read_float(chip->attr_gain);
  float aging    = attr_read_float(chip->attr_aging) / 100.0f;   // 0..1
  bool  cut      = attr_read_float(chip->attr_cut) >= 0.5f;

  // Laser off when: TX_DIS pin high (SFP), soft disable (SFP), or for CMIS when the data path
  // is not activated or the host disabled the lane output
  bool tx_disabled = pin_read(chip->pin_txdis) == HIGH;
  if (chip->type == 0) tx_disabled |= (chip->a2[110] & 0x40) != 0;
  else tx_disabled |= chip->dp_state != DP_ACTIVATED || (chip->p10[P10_TX_DISABLE] & 0x01);

  // Laser: the power-control loop raises bias current as the laser heats up and ages,
  // and an old laser still loses output power.
  float bias = 35.0f + fmaxf(0.0f, temp - 25.0f) * 0.3f + aging * 50.0f;
  float tx   = tx_set - aging * 8.0f + noise_db();
  if (tx_disabled) { tx = DARK_DBM; bias = 0.0f; }

  // Loopback fiber link: fiber + 2 connectors + 2 splices + bend, then optional amplifier
  float link_loss = length * FIBER_LOSS_DB_PER_KM + 2 * CONNECTOR_LOSS_DB + 2 * SPLICE_LOSS_DB + bend;
  float rx = tx - link_loss + gain + noise_db();
  bool dark = tx_disabled || cut;
  if (dark) rx = DARK_DBM;
  if (rx < DARK_DBM) rx = DARK_DBM;

  // Light levels for '1' and '0': same average power, ratio set by the extinction ratio
  float pavg_mw = dark ? 0.0f : powf(10.0f, rx / 10.0f);
  float er = powf(10.0f, attr_read_float(chip->attr_er) / 10.0f);
  chip->p1_mw = 2.0f * pavg_mw * er / (er + 1.0f);
  chip->p0_mw = 2.0f * pavg_mw / (er + 1.0f);

  // LOS with hysteresis: prevents the alarm from flapping around the threshold
  if (!chip->los && rx <= LOS_ASSERT_DBM) chip->los = true;
  else if (chip->los && rx > LOS_DEASSERT_DBM) chip->los = false;

  if (chip->type == 0) {
    put16(chip->live, 0, enc_temp(temp));
    put16(chip->live, 2, enc_vcc(3.30f));
    put16(chip->live, 4, enc_bias(bias));
    put16(chip->live, 6, enc_dbm(tx));
    put16(chip->live, 8, enc_dbm(rx));
    chip->live_status = (tx_disabled ? 0x80 : 0) | (chip->los ? 0x02 : 0);
    pin_write(chip->pin_los, chip->los ? HIGH : LOW);           // LOS pin, active high
    return;
  }

  // ---- CMIS: monitors and latched flags (a flag stays set until the host reads it) ----
  uint8_t *lo = chip->lower, *p11 = chip->p11;
  bool powered = chip->mod_state == ST_READY || chip->mod_state == ST_PWRUP;
  put16(lo, CMIS_TEMP, enc_temp(temp));
  put16(lo, CMIS_VCC, enc_vcc(3.30f));
  put16(p11, P11_TXPWR, enc_dbm(tx));
  put16(p11, P11_TXBIAS, enc_bias(bias));
  put16(p11, P11_RXPWR, enc_dbm(rx));

  lo[CMIS_TEMPVCC_FLAGS] |= limit_flags(temp, TEMP_LIMITS) | (limit_flags(3.30f, VCC_LIMITS) << 4);
  bool lane_live = powered && chip->dp_state == DP_ACTIVATED;   // lane flags only mean something on a live lane
  bool tx_on = lane_live && !(chip->p10[P10_TX_DISABLE] & 0x01) && pin_read(chip->pin_txdis) == LOW;
  if (tx_on) {                                       // no Tx power/bias alarms while the host has the laser off
    uint8_t tf = limit_flags(tx, TXPWR_LIMITS), bf = limit_flags(bias, BIAS_LIMITS);
    p11[P11_TXPWR_HA + 0] |= tf & 1;  p11[P11_TXPWR_HA + 1] |= (tf >> 1) & 1;   // Tx power HA, LA
    p11[P11_TXPWR_HA + 2] |= bf & 1;  p11[P11_TXPWR_HA + 3] |= (bf >> 1) & 1;   // Tx bias HA, LA
    p11[P11_TXPWR_HA + 4] |= (tf >> 2) & 1; p11[P11_TXPWR_HA + 5] |= (tf >> 3) & 1;   // Tx power HW, LW
    p11[P11_TXPWR_HA + 6] |= (bf >> 2) & 1; p11[P11_TXPWR_HA + 7] |= (bf >> 3) & 1;   // Tx bias HW, LW
  }
  if (lane_live) {
    uint8_t rf = limit_flags(rx, RXPWR_LIMITS);
    if (chip->los) p11[P11_RX_LOS] |= 0x01;
    for (int i = 0; i < 4; i++) p11[P11_RXPWR_HA + i] |= (rf >> i) & 1;
  }

  bool intl = cmis_any_flag(chip);                   // IntL asserted while any latched flag is set
  lo[CMIS_STATE] = (uint8_t)((chip->mod_state << 1) | (intl ? 0 : 1));
  pin_write(chip->pin_los, intl ? LOW : HIGH);       // pin used as IntL, active low
}

// ---------- Runs every 50 us: optical receiver output (analog) ----------
static void update_rx_signal(void *user_data) {
  chip_state_t *chip = user_data;
  bool bit = pin_read(chip->pin_txdata) == HIGH;                 // bit currently on the laser
  float p = bit ? chip->p1_mw : chip->p0_mw;
  float target = RX_OFFSET_V + RX_GAIN_V_PER_MW * p;
  if (target > RX_RAIL_V) target = RX_RAIL_V;                    // too much light: amplifier saturates

  float tau = attr_read_float(chip->attr_tau);                   // limited bandwidth: bits smear (ISI)
  float alpha = (tau <= 1.0f) ? 1.0f : 1.0f - expf(-RX_TICK_US / tau);
  chip->v_filt += (target - chip->v_filt) * alpha;

  float v = chip->v_filt + gauss() * attr_read_float(chip->attr_noise) / 1000.0f;   // receiver noise
  if (v < 0.0f) v = 0.0f;
  if (v > 3.3f) v = 3.3f;
  pin_dac_write(chip->pin_rxout, v);
}

// ---------- I2C device callbacks ----------
static bool on_connect(void *user_data, uint32_t address, bool read) {
  chip_state_t *chip = user_data;
  chip->cmis_txn = false;
  chip->page = NULL;
  if (chip->type == 1) {
    if (address != ADDR_A0) return false;            // CMIS answers only at 0x50
    chip->cmis_txn = true;
  } else if (address == ADDR_A0) {
    chip->page = chip->a0; chip->page_is_a2 = false;
  } else if (address == ADDR_A2) {
    chip->page = chip->a2; chip->page_is_a2 = true;
    memcpy(&chip->a2[96], chip->live, 10);                                   // latch a consistent snapshot
    chip->a2[110] = (chip->a2[110] & 0x40) | (chip->live_status & ~0x40);    // keep host's soft-disable bit
  } else {
    return false;                                    // not our address: NACK
  }
  if (!read) chip->expect_offset = true;             // first written byte sets the address pointer
  return true;
}

static bool cmis_is_latched(chip_state_t *chip, uint8_t addr) {
  if (addr >= CMIS_MOD_FLAGS && addr <= 13) return true;
  return addr >= P11_DP_CHANGED && addr <= 153 && chip->lower[CMIS_BANK_SEL] == 0
         && chip->lower[CMIS_PAGE_SEL] == 0x11;
}

static uint8_t on_read(void *user_data) {
  chip_state_t *chip = user_data;
  uint8_t addr = chip->ptr++;
  if (chip->cmis_txn) {
    uint8_t *m = addr < 128 ? chip->lower : cmis_upper_page(chip);
    if (!m) return 0;
    uint8_t v = m[addr];
    if (cmis_is_latched(chip, addr)) m[addr] = 0;    // latched flags clear on read
    return v;
  }
  if (!chip->page) return 0xFF;
  return chip->page[addr];
}

static bool on_write(void *user_data, uint8_t data) {
  chip_state_t *chip = user_data;
  if (chip->expect_offset) { chip->ptr = data; chip->expect_offset = false; return true; }
  uint8_t addr = chip->ptr++;

  if (chip->cmis_txn) {
    if (addr == CMIS_MOD_CTRL || addr == CMIS_BANK_SEL || addr == CMIS_PAGE_SEL ||
        (addr >= 31 && addr <= 36)) {                // control, bank/page select, flag masks
      chip->lower[addr] = data;
    } else if (addr >= 128 && chip->lower[CMIS_BANK_SEL] == 0 && chip->lower[CMIS_PAGE_SEL] == 0x10) {
      chip->p10[addr] = data;                        // page 10h: lane controls
    }
    return true;                                     // writes to read-only bytes are ignored
  }

  if (!chip->page) return false;
  if (chip->page_is_a2 && addr == 110)
    chip->a2[110] = (chip->a2[110] & ~0x40) | (data & 0x40);                 // soft TX disable bit
  else if (chip->page_is_a2 && addr >= 128 && addr <= 247)
    chip->a2[addr] = data;                                                   // user-writable EEPROM
  return true;
}

void chip_init(void) {
  chip_state_t *chip = calloc(1, sizeof(chip_state_t));
  chip->v_filt = RX_OFFSET_V;

  chip->attr_temp   = attr_init_float("temperature", 35.0f);
  chip->attr_tx     = attr_init_float("txPower", -1.0f);
  chip->attr_length = attr_init_float("fiberLength", 40.0f);
  chip->attr_bend   = attr_init_float("bendLoss", 0.0f);
  chip->attr_gain   = attr_init_float("ampGain", 0.0f);
  chip->attr_aging  = attr_init_float("laserAging", 0.0f);
  chip->attr_cut    = attr_init_float("fiberCut", 0.0f);
  chip->attr_er     = attr_init_float("extinctionRatio", 10.0f);
  chip->attr_tau    = attr_init_float("rxTimeConstant", 50.0f);
  chip->attr_noise  = attr_init_float("rxNoise", 7.0f);
  chip->attr_type   = attr_init_float("moduleType", 0.0f);

  chip->pin_los    = pin_init("LOS", OUTPUT_LOW);
  chip->pin_txdis  = pin_init("TX_DIS", INPUT_PULLDOWN);
  chip->pin_txdata = pin_init("TX_DATA", INPUT_PULLDOWN);
  chip->pin_rxout  = pin_init("RX_OUT", ANALOG);

  set_type(chip, attr_read_float(chip->attr_type) >= 0.5f ? 1 : 0);

  const i2c_config_t i2c = {
    .user_data = chip,
    .address = 0,                        // 0 = see all addresses; we answer only 0x50 (and 0x51 for SFP)
    .scl = pin_init("SCL", INPUT),
    .sda = pin_init("SDA", INPUT),
    .connect = on_connect,
    .read = on_read,
    .write = on_write,
  };
  i2c_init(&i2c);

  const timer_config_t tcfg = { .callback = update_monitors, .user_data = chip };
  chip->timer = timer_init(&tcfg);
  update_monitors(chip);
  timer_start_ns(chip->timer, 10000000, true);      // module logic + physics every 10 ms

  const timer_config_t rcfg = { .callback = update_rx_signal, .user_data = chip };
  chip->rx_timer = timer_init(&rcfg);
  timer_start_ns(chip->rx_timer, RX_TICK_NS, true); // receiver output every 50 us

  printf("Chip ready: SFP (SFF-8472) or QSFP-DD (CMIS) + loopback link + optical receiver\n");
}
