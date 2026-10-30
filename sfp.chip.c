// sfp.chip.c - Simulated SFP transceiver (SFF-8472) + fiber loopback link + optical receiver, for Wokwi
#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define ADDR_A0 0x50            // Serial ID page  ("A0h" in 8-bit notation)
#define ADDR_A2 0x51            // Diagnostics page ("A2h" in 8-bit notation)

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

typedef struct {
  uint8_t a0[256];
  uint8_t a2[256];
  uint8_t live[10];       // newest diagnostics, latched into a2 when a read starts
  uint8_t live_status;
  uint8_t *page;          // page addressed by the current I2C transaction
  bool page_is_a2;
  uint8_t ptr;            // EEPROM-style address pointer (wraps at 256)
  bool expect_offset;
  bool los;               // current LOS state (with hysteresis)
  float p1_mw, p0_mw;     // received optical power for a '1' and a '0'
  float v_filt;           // receiver output before noise (bandwidth-limited)
  uint32_t attr_temp, attr_tx, attr_length, attr_bend, attr_gain, attr_aging, attr_cut;
  uint32_t attr_er, attr_tau, attr_noise;
  pin_t pin_los, pin_txdis, pin_txdata, pin_rxout;
  timer_t timer, rx_timer;
} chip_state_t;

// ---------- SFF-8472 encoding helpers ----------
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
static void put_limits(uint8_t *a2, int off, uint16_t ha, uint16_t la, uint16_t hw, uint16_t lw) {
  put16(a2, off, ha); put16(a2, off + 2, la); put16(a2, off + 4, hw); put16(a2, off + 6, lw);
}

// ---------- Memory map setup ----------
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

// Small measurement noise on the diagnostics, +/- 0.05 dB
static float noise_db(void) { return ((rand() % 21) - 10) * 0.005f; }

// Gaussian random number (mean 0, std dev 1), Box-Muller method
static float gauss(void) {
  float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
  float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
  return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

// ---------- Runs every 10 ms: laser + fiber physics + diagnostics ----------
static void update_monitors(void *user_data) {
  chip_state_t *chip = user_data;
  float temp     = attr_read_float(chip->attr_temp);
  float tx_set   = attr_read_float(chip->attr_tx);
  float length   = attr_read_float(chip->attr_length);
  float bend     = attr_read_float(chip->attr_bend);
  float gain     = attr_read_float(chip->attr_gain);
  float aging    = attr_read_float(chip->attr_aging) / 100.0f;   // 0..1
  bool  cut      = attr_read_float(chip->attr_cut) >= 0.5f;

  bool soft_disable = chip->a2[110] & 0x40;
  bool tx_disabled  = (pin_read(chip->pin_txdis) == HIGH) || soft_disable;

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

  put16(chip->live, 0, enc_temp(temp));
  put16(chip->live, 2, enc_vcc(3.30f));
  put16(chip->live, 4, enc_bias(bias));
  put16(chip->live, 6, enc_dbm(tx));
  put16(chip->live, 8, enc_dbm(rx));

  chip->live_status = (tx_disabled ? 0x80 : 0) | (chip->los ? 0x02 : 0);
  pin_write(chip->pin_los, chip->los ? HIGH : LOW);
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
  if (address == ADDR_A0) {
    chip->page = chip->a0; chip->page_is_a2 = false;
  } else if (address == ADDR_A2) {
    chip->page = chip->a2; chip->page_is_a2 = true;
    memcpy(&chip->a2[96], chip->live, 10);                                   // latch a consistent snapshot
    chip->a2[110] = (chip->a2[110] & 0x40) | (chip->live_status & ~0x40);    // keep host's soft-disable bit
  } else {
    chip->page = NULL;
    return false;                        // not our address: NACK
  }
  if (!read) chip->expect_offset = true; // first written byte sets the address pointer
  return true;
}

static uint8_t on_read(void *user_data) {
  chip_state_t *chip = user_data;
  if (!chip->page) return 0xFF;
  return chip->page[chip->ptr++];
}

static bool on_write(void *user_data, uint8_t data) {
  chip_state_t *chip = user_data;
  if (!chip->page) return false;
  if (chip->expect_offset) { chip->ptr = data; chip->expect_offset = false; return true; }
  if (chip->page_is_a2 && chip->ptr == 110)
    chip->a2[110] = (chip->a2[110] & ~0x40) | (data & 0x40);                 // soft TX disable bit
  else if (chip->page_is_a2 && chip->ptr >= 128 && chip->ptr <= 247)
    chip->a2[chip->ptr] = data;                                              // user-writable EEPROM
  chip->ptr++;
  return true;
}

void chip_init(void) {
  chip_state_t *chip = calloc(1, sizeof(chip_state_t));
  init_a0(chip->a0);
  init_a2(chip->a2);
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

  chip->pin_los    = pin_init("LOS", OUTPUT_LOW);
  chip->pin_txdis  = pin_init("TX_DIS", INPUT_PULLDOWN);
  chip->pin_txdata = pin_init("TX_DATA", INPUT_PULLDOWN);
  chip->pin_rxout  = pin_init("RX_OUT", ANALOG);

  const i2c_config_t i2c = {
    .user_data = chip,
    .address = 0,                        // 0 = see all addresses; we answer only 0x50/0x51
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
  timer_start_ns(chip->timer, 10000000, true);      // physics + diagnostics every 10 ms

  const timer_config_t rcfg = { .callback = update_rx_signal, .user_data = chip };
  chip->rx_timer = timer_init(&rcfg);
  timer_start_ns(chip->rx_timer, RX_TICK_NS, true); // receiver output every 50 us

  printf("SFP chip ready: loopback link + optical receiver (A0h=0x50, A2h=0x51)\n");
}