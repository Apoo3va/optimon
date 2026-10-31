// test_main.cpp - PC unit tests for the OptiMon firmware logic.
// Build: g++ -std=c++17 -Wall -Wextra -Werror -I . tests/test_main.cpp -o run_tests
#include <cmath>
#include <cstdio>
#include <cstring>
#include "sff8472.h"
#include "prbs7.h"
#include "frame.h"
#include "cmis.h"

static int g_checks = 0, g_failures = 0;

#define CHECK(cond) do { g_checks++; if (!(cond)) { g_failures++; \
  std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_NEAR(a, b, tol) do { g_checks++; double _a = (a), _b = (b); \
  if (std::fabs(_a - _b) > (tol)) { g_failures++; \
  std::printf("  FAIL %s:%d: %s = %.4f, expected %.4f\n", __FILE__, __LINE__, #a, _a, _b); } } while (0)

// ---- Encoders that mirror the simulated module (sfp.chip.c) ----
static uint16_t encTemp(float c) { return (uint16_t)(int16_t)std::lround(c * 256.0f); }
static uint16_t encVcc(float v)  { return (uint16_t)std::lround(v / 100e-6f); }
static uint16_t encBias(float mA){ return (uint16_t)std::lround(mA / 2e-3f); }
static uint16_t encDbm(float dbm){ return (uint16_t)std::lround(std::pow(10.0f, dbm / 10.0f) * 1e4f); }
static void put16(uint8_t *m, int off, uint16_t v) { m[off] = v >> 8; m[off + 1] = v & 0xFF; }
static void putLimits(uint8_t *m, int off, uint16_t ha, uint16_t la, uint16_t hw, uint16_t lw) {
  put16(m, off, ha); put16(m, off + 2, la); put16(m, off + 4, hw); put16(m, off + 6, lw);
}

// ================= SFF-8472 decoder =================
static void testBigEndian() {
  const uint8_t b[2] = {0x12, 0x34};
  CHECK(sff::be16(b) == 0x1234);
}

static void testOpticalPower() {
  CHECK_NEAR(sff::toDbm(1000), -10.0, 1e-4);    // 100 uW
  CHECK_NEAR(sff::toDbm(10000), 0.0, 1e-4);     // 1 mW
  CHECK_NEAR(sff::toDbm(1), -40.0, 1e-4);       // 0.1 uW, smallest step
  CHECK_NEAR(sff::toDbm(0), -40.0, 1e-4);       // no light: clamped, no log(0)
}

static void testScalarDecoding() {
  CHECK_NEAR(sff::decode(sff::P_TEMP, encTemp(35.0f)), 35.0, 1e-3);
  CHECK_NEAR(sff::decode(sff::P_TEMP, encTemp(-5.0f)), -5.0, 1e-3);   // signed value
  CHECK_NEAR(sff::decode(sff::P_TEMP, 0x8000), -128.0, 1e-3);         // most negative
  CHECK_NEAR(sff::decode(sff::P_VCC, 33000), 3.30, 1e-4);
  CHECK_NEAR(sff::decode(sff::P_BIAS, 17500), 35.0, 1e-3);
}

static void testRxThresholdQuantization() {
  // -24 dBm is 39.81 steps of 0.1 uW, stored as 40, which reads back as -23.98 dBm
  CHECK(encDbm(-24.0f) == 40);
  CHECK_NEAR(sff::toDbm(encDbm(-24.0f)), -23.98, 0.01);
  // Near -30 dBm one LSB is about 0.46 dB
  CHECK_NEAR(sff::toDbm(10) - sff::toDbm(9), 0.46, 0.01);
}

static void testChecksum() {
  uint8_t m[64] = {};
  for (int i = 0; i < 63; i++) m[i] = (uint8_t)(i * 7 + 3);
  uint32_t s = 0;
  for (int i = 0; i < 63; i++) s += m[i];
  m[63] = (uint8_t)(s & 0xFF);
  CHECK(sff::checksumOk(m, 0, 62, m[63]));
  m[10] ^= 0x01;                                  // one corrupted bit
  CHECK(!sff::checksumOk(m, 0, 62, m[63]));
}

static void testMonitorsAndLimits() {
  uint8_t a2[256] = {};
  putLimits(a2, 0,  encTemp(75), encTemp(-5), encTemp(70), encTemp(0));
  putLimits(a2, 8,  encVcc(3.6f), encVcc(3.0f), encVcc(3.5f), encVcc(3.1f));
  putLimits(a2, 16, encBias(80), encBias(2), encBias(70), encBias(4));
  putLimits(a2, 24, encDbm(4), encDbm(-8), encDbm(3), encDbm(-7));
  putLimits(a2, 32, encDbm(0), encDbm(-24), encDbm(-1), encDbm(-22));
  put16(a2, 96, encTemp(35)); put16(a2, 98, encVcc(3.3f)); put16(a2, 100, encBias(38));
  put16(a2, 102, encDbm(-1)); put16(a2, 104, encDbm(-10));

  sff::Limits t = sff::decodeLimits(a2 + sff::A2_THRESHOLDS, sff::P_TEMP);
  CHECK_NEAR(t.highAlarm, 75.0, 1e-3);
  CHECK_NEAR(t.lowAlarm, -5.0, 1e-3);
  sff::Limits rx = sff::decodeLimits(a2 + sff::A2_THRESHOLDS, sff::P_RX);
  CHECK_NEAR(rx.lowAlarm, -23.98, 0.01);
  CHECK_NEAR(rx.lowWarn, -22.01, 0.01);

  const uint8_t *mon = a2 + sff::A2_MONITORS;
  CHECK_NEAR(sff::decodeMonitor(mon, sff::P_TEMP), 35.0, 1e-3);
  CHECK_NEAR(sff::decodeMonitor(mon, sff::P_VCC), 3.30, 1e-3);
  CHECK_NEAR(sff::decodeMonitor(mon, sff::P_BIAS), 38.0, 1e-3);
  CHECK_NEAR(sff::decodeMonitor(mon, sff::P_TX), -1.0, 0.01);
  CHECK_NEAR(sff::decodeMonitor(mon, sff::P_RX), -10.0, 0.01);
}

static void testClassify() {
  const sff::Limits rx = {0.0f, -23.98f, -1.0f, -22.01f};
  CHECK(sff::classify(-10.0f, rx) == sff::L_OK);
  CHECK(sff::classify(-23.0f, rx) == sff::L_WARN_LOW);
  CHECK(sff::classify(-26.0f, rx) == sff::L_ALARM_LOW);
  CHECK(sff::classify(-0.5f, rx) == sff::L_WARN_HIGH);
  CHECK(sff::classify(7.8f, rx) == sff::L_ALARM_HIGH);     // receiver overload
  CHECK(sff::classify(-22.01f, rx) == sff::L_OK);          // exactly on a limit is not past it
  CHECK(sff::isAlarm(sff::L_ALARM_LOW));
  CHECK(!sff::isAlarm(sff::L_WARN_HIGH));
  CHECK(sff::L_ALARM_HIGH > sff::L_WARN_HIGH);             // severity order used for "worst"
  CHECK(std::strcmp(sff::levelName(sff::L_ALARM_LOW), "LO-A") == 0);
}

static void testAsciiField() {
  const uint8_t raw[16] = {'A','P','O','O','R','V','A',' ','O','P','T','I','C','S',' ',' '};
  char out[17];
  sff::asciiField(out, raw, 16);
  CHECK(std::strcmp(out, "APOORVA OPTICS") == 0);          // inner space kept, padding trimmed
}

// ================= PRBS-7 =================
static void testPrbsPeriodAndBalance() {
  uint8_t s = 0x7F, start = s;
  int ones = 0, period = 0;
  do { ones += prbs7Next(s); period++; CHECK(s != 0); } while (s != start && period < 1000);
  CHECK(period == 127);                                     // maximal length: 2^7 - 1
  CHECK(ones == 64);                                        // 64 ones and 63 zeros per period
}

static void testPrbsAllStatesVisited() {
  bool seen[128] = {};
  uint8_t s = 0x01;
  for (int i = 0; i < 127; i++) { seen[s] = true; prbs7Next(s); }
  int count = 0;
  for (int v = 1; v < 128; v++) count += seen[v];
  CHECK(count == 127);                                      // every non-zero state exactly once
  CHECK(!seen[0]);
}

// Checker as in the firmware: load 7 received bits into its register, then predict every next bit
struct Checker {
  uint8_t reg = 0; int loaded = 0;
  bool locked = false; long bits = 0, errors = 0;
  void feed(uint8_t rx) {
    if (!locked) { reg = (uint8_t)(((reg << 1) | rx) & 0x7F); if (++loaded >= 7) locked = reg != 0; return; }
    uint8_t expected = prbs7Next(reg);
    bits++;
    if (rx != expected) errors++;
  }
};

static void testCheckerSelfSyncsAtAnyOffset() {
  for (int offset = 0; offset < 127; offset += 13) {
    uint8_t tx = 0x7F;
    for (int i = 0; i < offset; i++) prbs7Next(tx);        // receiver joins mid-sequence
    Checker c;
    for (int i = 0; i < 1000; i++) c.feed(prbs7Next(tx));
    CHECK(c.locked);
    CHECK(c.bits == 993);
    CHECK(c.errors == 0);
  }
}

static void testCheckerCountsEachErrorOnce() {
  uint8_t tx = 0x7F;
  Checker c;
  const int flips[] = {100, 250, 251, 600};                // includes two errors in a row
  int k = 0;
  for (int i = 0; i < 1000; i++) {
    uint8_t b = prbs7Next(tx);
    if (k < 4 && i == flips[k]) { b ^= 1; k++; }
    c.feed(b);
  }
  CHECK(c.errors == 4);                                     // no error multiplication
}

static void testNoSignalNeverLocks() {
  Checker c;
  for (int i = 0; i < 100; i++) c.feed(0);                  // dark receiver: all zeros
  CHECK(!c.locked);
}


// ================= Framing and CRC-16 =================
static void sendFrameBits(frm::Receiver &rx, const uint8_t *f, int n, int flipBit = -1) {
  for (int i = 0; i < n * 8; i++) {
    uint8_t b = (f[i >> 3] >> (7 - (i & 7))) & 1;
    if (i == flipBit) b ^= 1;
    rx.push(b);
  }
}

static void testCrc16() {
  const uint8_t check[] = {'1','2','3','4','5','6','7','8','9'};
  CHECK(frm::crc16(check, 9) == 0x29B1);                   // standard CRC-16/CCITT-FALSE check value
  CHECK(frm::crc16(check, 0) == 0xFFFF);
}

static void testFrameLayout() {
  uint8_t f[frm::MAX_FRAME_BYTES];
  int n = frm::build(f, 0x0102, "HI", 2);
  CHECK(n == 4 + 2 + 3 + 2 + 2);
  CHECK(f[0] == 0x55 && f[3] == 0x55);
  CHECK(f[4] == 0x2D && f[5] == 0xD4);
  CHECK(f[6] == 0x01 && f[7] == 0x02 && f[8] == 2);
  CHECK(f[9] == 'H' && f[10] == 'I');
  CHECK(frm::build(f, 0, "x", frm::MAX_PAYLOAD + 1) == 0); // too long: rejected
}

static void testFrameRoundTrip() {
  frm::Receiver rx;
  uint8_t f[frm::MAX_FRAME_BYTES];
  for (uint16_t seq = 0; seq < 20; seq++) {
    char text[24];
    int len = std::snprintf(text, sizeof(text), "OPTIMON #%05u", (unsigned)seq);
    int n = frm::build(f, seq, text, len);
    sendFrameBits(rx, f, n);
  }
  CHECK(rx.good == 20);
  CHECK(rx.crcErrors == 0 && rx.missed == 0);
  CHECK(std::strcmp(rx.lastText, "OPTIMON #00019") == 0);
}

static void testFrameJoinMidStream() {
  frm::Receiver rx;
  uint8_t f[frm::MAX_FRAME_BYTES];
  int n = frm::build(f, 7, "OPTIMON #00007", 14);
  for (int i = 37; i < n * 8; i++) rx.push((f[i >> 3] >> (7 - (i & 7))) & 1);   // start mid-frame
  CHECK(rx.good == 0);
  n = frm::build(f, 8, "OPTIMON #00008", 14);
  sendFrameBits(rx, f, n);
  CHECK(rx.good == 1);                                      // locks on the next sync word
  CHECK(std::strcmp(rx.lastText, "OPTIMON #00008") == 0);
}

static void testFrameBitErrorCaught() {
  uint8_t f[frm::MAX_FRAME_BYTES];
  int n = frm::build(f, 3, "OPTIMON #00003", 14);
  int detected = 0;
  for (int bit = 6 * 8; bit < n * 8; bit++) {               // every bit after the sync word
    frm::Receiver rx;
    sendFrameBits(rx, f, n, bit);
    if (rx.good == 0) detected++;
  }
  CHECK(detected == (n - 6) * 8);                           // every single-bit error is detected
}

static void testFrameMissedCount() {
  frm::Receiver rx;
  uint8_t f[frm::MAX_FRAME_BYTES];
  const uint16_t seqs[] = {1, 2, 5, 6};                     // frames 3 and 4 never arrive
  for (uint16_t s : seqs) {
    int n = frm::build(f, s, "DATA", 4);
    sendFrameBits(rx, f, n);
  }
  CHECK(rx.good == 4);
  CHECK(rx.missed == 2);
}

static void testCorruptFrameCountedOnce() {
  frm::Receiver rx;
  uint8_t f[frm::MAX_FRAME_BYTES];
  for (uint16_t s = 1; s <= 3; s++) {
    int n = frm::build(f, s, "DATA", 4);
    sendFrameBits(rx, f, n, s == 2 ? 80 : -1);             // frame 2 arrives with one bad bit
  }
  CHECK(rx.good == 2);
  CHECK(rx.crcErrors == 1);
  CHECK(rx.missed == 0);                                    // the gap at seq 2 is that CRC error, not a loss
}

static void testFrameBadLengthRejected() {
  frm::Receiver rx;
  const uint8_t junk[] = {0x2D, 0xD4, 0x00, 0x01, 0xFF};   // sync, seq, length 255
  sendFrameBits(rx, junk, 5);
  CHECK(rx.lengthErrors == 1);
  CHECK(rx.hunting);                                        // back to hunting for sync
}


// ================= CMIS decoders =================
static void testCmisIdentifiers() {
  CHECK(cmis::isCmisId(0x18));                              // QSFP-DD
  CHECK(cmis::isCmisId(0x19));                              // OSFP
  CHECK(cmis::isCmisId(0x1E));                              // QSFP+ with CMIS
  CHECK(!cmis::isCmisId(0x03));                             // SFP uses SFF-8472
  CHECK(!cmis::isCmisId(0x11));                             // QSFP28 uses SFF-8636
}

static void testCmisModuleState() {
  CHECK(cmis::moduleState(0x03) == cmis::MS_LOWPWR);        // state 1, IntL not asserted
  CHECK(cmis::moduleState(0x07) == cmis::MS_READY);         // state 3
  CHECK(cmis::moduleState(0x06) == cmis::MS_READY);
  CHECK(cmis::intlAsserted(0x06));                          // bit 0 = 0 means IntL asserted
  CHECK(!cmis::intlAsserted(0x07));
  CHECK(cmis::moduleState(0x0E) == cmis::MS_UNKNOWN);       // 7 is not a defined state
  CHECK(std::strcmp(cmis::moduleStateName(cmis::MS_PWRUP), "ModulePwrUp") == 0);
}

static void testCmisDataPathNibbles() {
  const uint8_t dp[4] = {0x14, 0x21, 0x11, 0x71};           // lane 1 = 4, lane 2 = 1, lane 3 = 1, lane 4 = 2
  CHECK(cmis::dpState(dp, 1) == cmis::DP_ACTIVATED);
  CHECK(cmis::dpState(dp, 2) == cmis::DP_DEACTIVATED);
  CHECK(cmis::dpState(dp, 4) == cmis::DP_INIT);
  CHECK(cmis::dpState(dp, 7) == cmis::DP_DEACTIVATED);
  CHECK(cmis::dpState(dp, 8) == cmis::DP_INITIALIZED);
}

static void testCmisThresholdOrder() {
  uint8_t p02[128] = {};                                    // page 02h bytes 128..255
  putLimits(p02, cmis::P02_TEMP - 128,   encTemp(75), encTemp(-5), encTemp(70), encTemp(0));
  putLimits(p02, cmis::P02_TXPWR - 128,  encDbm(4), encDbm(-8), encDbm(3), encDbm(-7));
  putLimits(p02, cmis::P02_TXBIAS - 128, encBias(80), encBias(2), encBias(70), encBias(4));
  putLimits(p02, cmis::P02_RXPWR - 128,  encDbm(0), encDbm(-24), encDbm(-1), encDbm(-22));
  CHECK_NEAR(cmis::limitsFromPage02(p02, sff::P_TEMP).lowAlarm, -5.0, 1e-3);
  CHECK_NEAR(cmis::limitsFromPage02(p02, sff::P_BIAS).highAlarm, 80.0, 1e-3);   // bias sits after Tx power
  CHECK_NEAR(cmis::limitsFromPage02(p02, sff::P_TX).lowWarn, -7.0, 0.01);
  CHECK_NEAR(cmis::limitsFromPage02(p02, sff::P_RX).lowAlarm, -23.98, 0.01);
}

int main() {
  struct { const char *name; void (*fn)(); } tests[] = {
    {"big-endian read", testBigEndian},
    {"optical power dBm", testOpticalPower},
    {"temperature / Vcc / bias", testScalarDecoding},
    {"Rx threshold quantization", testRxThresholdQuantization},
    {"checksum", testChecksum},
    {"monitors and limits", testMonitorsAndLimits},
    {"alarm classification", testClassify},
    {"ASCII field trim", testAsciiField},
    {"PRBS-7 period and balance", testPrbsPeriodAndBalance},
    {"PRBS-7 visits all states", testPrbsAllStatesVisited},
    {"checker self-sync at any offset", testCheckerSelfSyncsAtAnyOffset},
    {"checker counts each error once", testCheckerCountsEachErrorOnce},
    {"no signal never locks", testNoSignalNeverLocks},
    {"CRC-16 check value", testCrc16},
    {"frame layout", testFrameLayout},
    {"frame round trip", testFrameRoundTrip},
    {"frame receiver joins mid-stream", testFrameJoinMidStream},
    {"every single-bit error caught", testFrameBitErrorCaught},
    {"missed frames counted", testFrameMissedCount},
    {"corrupt frame counted once", testCorruptFrameCountedOnce},
    {"bad length rejected", testFrameBadLengthRejected},
    {"CMIS identifiers", testCmisIdentifiers},
    {"CMIS module state byte", testCmisModuleState},
    {"CMIS data path nibbles", testCmisDataPathNibbles},
    {"CMIS page 02h threshold order", testCmisThresholdOrder},
  };
  for (auto &t : tests) {
    int before = g_failures;
    t.fn();
    std::printf("%s %s\n", g_failures == before ? "[PASS]" : "[FAIL]", t.name);
  }
  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
