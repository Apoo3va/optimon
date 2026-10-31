// test_chip.cpp - Tests for the simulated module itself (sfp.chip.c), run on a PC.
// Build: gcc -std=c11 -I tests -c sfp.chip.c tests/wokwi_stub.c && g++ -std=c++17 -I tests -I . tests/test_chip.cpp *.o
#include <cmath>
#include <cstdio>
#include <cstring>
#include "wokwi_stub.h"
#include "sff8472.h"
#include "cmis.h"

static int g_checks = 0, g_failures = 0;
#define CHECK(cond) do { g_checks++; if (!(cond)) { g_failures++; \
  std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

// ---- I2C helpers, as a host would do them ----
static bool rd(uint8_t addr, uint8_t off, uint8_t *buf, int n) {
  if (!sim_i2c_connect(addr, 0)) return false;
  sim_i2c_write(off);
  if (!sim_i2c_connect(addr, 1)) return false;
  for (int i = 0; i < n; i++) buf[i] = sim_i2c_read();
  return true;
}
static uint8_t rd1(uint8_t addr, uint8_t off) { uint8_t b = 0xEE; rd(addr, off, &b, 1); return b; }
static bool wr(uint8_t addr, uint8_t off, uint8_t v) {
  if (!sim_i2c_connect(addr, 0)) return false;
  sim_i2c_write(off);
  return sim_i2c_write(v);
}
static void page(uint8_t p) { wr(0x50, cmis::PAGE_SEL, p); }
static void ticks(int n) { for (int i = 0; i < n; i++) sim_tick_10ms(); }   // 10 ms each
static void setType(int t) { sim_attr_set("moduleType", (float)t); ticks(1); }

// ================= SFP mode =================
static void testSfpIdentity() {
  setType(0);
  uint8_t a0[96], a2[96];
  CHECK(rd(0x50, 0, a0, 96));
  CHECK(a0[0] == 0x03);
  CHECK(sff::checksumOk(a0, 0, 62, a0[63]));
  CHECK(sff::checksumOk(a0, 64, 94, a0[95]));
  CHECK(rd(0x51, 0, a2, 96));
  CHECK(sff::checksumOk(a2, 0, 94, a2[95]));
  CHECK(!sim_i2c_connect(0x52, 0));                         // nobody else on the bus
}

static void testSfpLosPin() {
  setType(0);
  sim_attr_set("fiberCut", 1); ticks(2);
  CHECK(sim_pin_get("LOS") == 1);                           // active high
  sim_attr_set("fiberCut", 0); ticks(2);
  CHECK(sim_pin_get("LOS") == 0);
}

// ================= CMIS mode =================
static void testCmisIdentityAndPaging() {
  setType(1);
  CHECK(rd1(0x50, 0) == cmis::ID_QSFP_DD);
  CHECK(!sim_i2c_connect(0x51, 0));                         // CMIS uses 0x50 only
  CHECK((rd1(0x50, cmis::MEM_MODEL) & cmis::MEM_FLAT) == 0);   // paged memory
  page(0x00);
  uint8_t p0[96];
  CHECK(rd(0x50, 128, p0, 96));
  CHECK(sff::checksumOk(p0, 0, 93, p0[cmis::P00_CHECKSUM - 128]));
  CHECK(std::memcmp(p0 + (cmis::P00_VENDOR_PN - 128), "QDD-SIM-ZR", 10) == 0);
  page(0x55);                                               // not implemented: reads as zeros
  CHECK(rd1(0x50, 128) == 0);
  page(0x00);
}

static void testCmisPowerUpSequence() {
  setType(0); setType(1);                                   // fresh module
  CHECK(cmis::moduleState(rd1(0x50, cmis::STATE)) == cmis::MS_LOWPWR);
  ticks(100);
  CHECK(cmis::moduleState(rd1(0x50, cmis::STATE)) == cmis::MS_LOWPWR);   // waits for the host
  wr(0x50, cmis::MOD_CTRL, rd1(0x50, cmis::MOD_CTRL) & ~cmis::CTRL_LOWPWR_SW);
  ticks(1);
  CHECK(cmis::moduleState(rd1(0x50, cmis::STATE)) == cmis::MS_PWRUP);
  ticks(198);
  CHECK(cmis::moduleState(rd1(0x50, cmis::STATE)) == cmis::MS_PWRUP);
  ticks(2);
  CHECK(cmis::moduleState(rd1(0x50, cmis::STATE)) == cmis::MS_READY);   // after 2.0 s

  page(0x11);
  uint8_t dp[4];
  rd(0x50, cmis::P11_DP_STATE, dp, 4);
  CHECK(cmis::dpState(dp, 1) == cmis::DP_DEACTIVATED);      // data path still held by DPDeinit
  page(0x10);
  wr(0x50, cmis::P10_DP_DEINIT, 0x00);
  ticks(2);
  page(0x11);
  rd(0x50, cmis::P11_DP_STATE, dp, 4);
  CHECK(cmis::dpState(dp, 1) == cmis::DP_INIT);
  ticks(100);
  rd(0x50, cmis::P11_DP_STATE, dp, 4);
  CHECK(cmis::dpState(dp, 1) == cmis::DP_ACTIVATED);
  CHECK(cmis::dpState(dp, 2) == cmis::DP_DEACTIVATED);
}

static void testCmisLaserFollowsDataPath() {
  // continues from the activated module above
  page(0x11);
  uint8_t m[2];
  rd(0x50, cmis::P11_TXPWR, m, 2);
  CHECK(sff::toDbm(sff::be16(m)) > -2.0f);                  // laser on, about -1 dBm
  page(0x10);
  wr(0x50, cmis::P10_TX_DISABLE, 0x01);                     // host disables lane 1 output
  ticks(1);
  page(0x11);
  rd(0x50, cmis::P11_TXPWR, m, 2);
  CHECK(sff::toDbm(sff::be16(m)) <= -39.9f);                // dark: the -40 dBm floor
  page(0x10);
  wr(0x50, cmis::P10_TX_DISABLE, 0x00);
  ticks(1);
}

static void testCmisLatchedFlagsAndIntL() {
  page(0x11);
  uint8_t f[20];
  rd(0x50, cmis::P11_DP_CHANGED, f, 20);                    // clear everything from the bring-up
  uint8_t mod[6];
  rd(0x50, cmis::MOD_FLAGS, mod, 6);
  ticks(1);
  CHECK(sim_pin_get("LOS") == 1);                           // IntL high = not asserted
  CHECK(!cmis::intlAsserted(rd1(0x50, cmis::STATE)));

  sim_attr_set("fiberCut", 1); ticks(1);
  CHECK(sim_pin_get("LOS") == 0);                           // IntL asserted (active low)
  sim_attr_set("fiberCut", 0); ticks(2);                    // light is back, but the flag is latched
  CHECK(sim_pin_get("LOS") == 0);
  CHECK(rd1(0x50, cmis::P11_RX_LOS) & 0x01);                // host reads the latched LOS ...
  CHECK(rd1(0x50, cmis::P11_RX_LOS) == 0);                  // ... and the read cleared it
  rd(0x50, cmis::P11_DP_CHANGED, f, 20);                    // clear the Rx power flags it also raised
  ticks(1);
  CHECK(sim_pin_get("LOS") == 1);                           // IntL released
}

static void testCmisLowPowerAgain() {
  wr(0x50, cmis::MOD_CTRL, rd1(0x50, cmis::MOD_CTRL) | cmis::CTRL_LOWPWR_SW);
  ticks(1);
  CHECK(cmis::moduleState(rd1(0x50, cmis::STATE)) == cmis::MS_PWRDN);
  page(0x11);
  uint8_t dp[4];
  rd(0x50, cmis::P11_DP_STATE, dp, 4);
  CHECK(cmis::dpState(dp, 1) == cmis::DP_DEACTIVATED);      // data path drops with the module
  ticks(50);
  CHECK(cmis::moduleState(rd1(0x50, cmis::STATE)) == cmis::MS_LOWPWR);
  CHECK(rd1(0x50, cmis::MOD_FLAGS) & cmis::FLAG_STATE_CHANGED);
}

static void testCmisReadOnlyBytes() {
  uint8_t before = rd1(0x50, 0);
  wr(0x50, 0, 0x99);                                        // identifier is read-only
  CHECK(rd1(0x50, 0) == before);
  page(0x11);
  wr(0x50, cmis::P11_DP_STATE, 0xFF);                       // status page is read-only
  uint8_t dp[1];
  rd(0x50, cmis::P11_DP_STATE, dp, 1);
  CHECK(dp[0] != 0xFF);
}

int main() {
  sim_chip_init();
  struct { const char *name; void (*fn)(); } tests[] = {
    {"SFP identity and checksums", testSfpIdentity},
    {"SFP LOS pin", testSfpLosPin},
    {"CMIS identity and paging", testCmisIdentityAndPaging},
    {"CMIS power-up sequence", testCmisPowerUpSequence},
    {"CMIS laser follows data path", testCmisLaserFollowsDataPath},
    {"CMIS latched flags and IntL", testCmisLatchedFlagsAndIntL},
    {"CMIS back to low power", testCmisLowPowerAgain},
    {"CMIS read-only bytes", testCmisReadOnlyBytes},
  };
  for (auto &t : tests) {
    int before = g_failures;
    t.fn();
    std::printf("%s %s\n", g_failures == before ? "[PASS]" : "[FAIL]", t.name);
  }
  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
