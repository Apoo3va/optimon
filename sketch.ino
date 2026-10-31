// OptiMon - optical module manager for SFP (SFF-8472) and QSFP-DD (CMIS) modules on ESP32:
// diagnostics + alarms, Automatic Laser Shutdown, PRBS-7 BER tester, framed data link (CRC-16).
// Serial commands: prbs | frames | rate <bit/s> | help
#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_timer.h"
#include "sff8472.h"
#include "cmis.h"
#include "prbs7.h"
#include "frame.h"

const int PIN_SDA = 21, PIN_SCL = 22;
const int PIN_LED_GREEN = 26, PIN_LED_RED = 27;
const int PIN_TX_DIS = 33;              // SFP: HIGH = laser off (QSFP-DD has no such pin; kept LOW)
const int PIN_LOS = 35;                 // SFP: LOS, active high. QSFP-DD: IntL, active low
const int PIN_TX_DATA = 25;             // bits to the laser
const int PIN_RX_ADC = 34;              // analog receiver output from the module
const uint32_t POLL_MS = 1000;          // diagnostics poll period
const uint32_t DIAG_PRINT_MS = 5000;    // how often diagnostics are printed
const uint32_t ALS_OFF_MS = 5000;       // laser stays off this long before a restart probe
const uint32_t ALS_PROBE_MS = 2000;     // laser on for this long to test the fiber
const uint32_t CMIS_STEP_MS = 50;       // CMIS host state machine / flag service period
const uint32_t CMIS_TIMEOUT_MS = 10000; // give up waiting for a module or data path state
const uint32_t DEFAULT_BIT_RATE = 1000; // bits per second
const uint32_t MIN_BIT_RATE = 100, MAX_BIT_RATE = 5000;
const uint32_t BERT_WINDOW_MS = 5000;   // measurement window
const float ADC_TO_MV = 3300.0f / 4095.0f;

enum AlsState : uint8_t { ALS_NORMAL, ALS_SHUTDOWN, ALS_PROBING };
const char *ALS_NAMES[] = {"ON", "OFF", "PROBE"};

enum LinkMode : uint8_t { MODE_PRBS, MODE_FRAMES };
const char *MODE_NAMES[] = {"PRBS-7 BER test", "framed data + CRC-16"};

enum ModuleKind : uint8_t { MOD_NONE, MOD_SFP, MOD_CMIS };

// Host-side CMIS bring-up: release low power, wait for ModuleReady, release the data path, wait for DPActivated
enum CmisHost : uint8_t { H_IDLE, H_RELEASE_LOWPWR, H_WAIT_READY, H_RELEASE_DP, H_WAIT_DP, H_UP, H_FAILED };

Adafruit_SSD1306 oled(128, 64, &Wire, -1);

const char *NAMES[sff::NUM_PARAMS] = {"Temp", "Vcc", "Bias", "TxPwr", "RxPwr"};
const char *UNITS[sff::NUM_PARAMS] = {" C", " V", " mA", " dBm", " dBm"};
const int   DECIMALS[sff::NUM_PARAMS] = {1, 2, 1, 2, 2};

sff::Limits limits[sff::NUM_PARAMS];
char vendor[17], partNumber[17];
uint16_t wavelengthNm = 0;
ModuleKind moduleKind = MOD_NONE;
uint8_t moduleId = 0;                   // SFF-8024 identifier of the inserted module

// CMIS host state (main loop only)
CmisHost cmisHost = H_IDLE;
cmis::ModuleState cmisState = cmis::MS_UNKNOWN;
cmis::DataPathState cmisDp = cmis::DP_UNKNOWN;
uint8_t cmisPage = 0xFF;                // page currently selected in the module (0xFF = unknown)
uint32_t cmisStepMs = 0, cmisPhaseMs = 0, cmisBringupMs = 0;
uint32_t cmisLosSeenMs = 0;             // last time the latched Rx LOS flag was seen set
bool cmisLosEver = false;

// ================= Data link types and shared state =================
// Defined before any function: the Arduino builder inserts function prototypes above the
// first function, and reportPrbs/reportFrames take a LinkStats parameter.
struct LinkStats {
  // PRBS mode
  uint32_t bits = 0, errors = 0, lockLosses = 0;
  uint32_t n1 = 0, n0 = 0;                     // samples of known '1's and '0's
  double sum1 = 0, sq1 = 0, sum0 = 0, sq0 = 0; // for mean and spread of each level
  // Frame mode
  uint32_t framesSent = 0, framesGood = 0, framesBad = 0, framesMissed = 0;
  char lastText[frm::MAX_PAYLOAD + 1] = "";
};
LinkStats linkAcc;                             // filled by the bit-clock task
portMUX_TYPE linkMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool linkActive = true;               // false while ALS has the laser off
volatile LinkMode linkMode = MODE_PRBS;
volatile bool linkResetRequest = false;        // set by the main loop, handled by the bit-clock task
uint32_t bitRate = DEFAULT_BIT_RATE;           // main loop only
esp_timer_handle_t bitTimer = nullptr;

struct LinkResult {                            // last window, for the OLED (main loop only)
  bool valid = false;
  LinkMode mode = MODE_PRBS;
  uint32_t bits = 0, errors = 0, frames = 0, frameErrors = 0;
  float ber = 0, q = 0, fer = 0;
};
LinkResult linkLast;

// ================= Automatic Laser Shutdown =================
// ---- Shared between the interrupt handlers and the main loop ----
volatile AlsState alsState = ALS_NORMAL;
volatile bool losEvent = false;         // SFP: the LOS ISR shut the laser off
volatile bool intlEvent = false;        // CMIS: IntL fell, flags need servicing
volatile uint32_t isrStartUs = 0, isrEndUs = 0, intlUs = 0;
portMUX_TYPE alsMux = portMUX_INITIALIZER_UNLOCKED;   // protects data shared with the ISRs

// ---- Main-loop-only ALS bookkeeping ----
uint32_t alsTimerMs = 0, outageStartMs = 0;
unsigned long probeCount = 0, outageCount = 0;

// SFP: LOS rising edge. Shut the laser off right here in the ISR (eye safety must not wait for the main loop)
void IRAM_ATTR onLosRising() {
  portENTER_CRITICAL_ISR(&alsMux);
  uint32_t t0 = micros();
  if (alsState == ALS_NORMAL) {
    digitalWrite(PIN_TX_DIS, HIGH);
    alsState = ALS_SHUTDOWN;
    isrStartUs = t0;
    isrEndUs = micros();
    losEvent = true;
  }
  portEXIT_CRITICAL_ISR(&alsMux);
}

// CMIS: IntL falling edge. The cause is in the latched flags, which need I2C, so the ISR only records it
void IRAM_ATTR onIntL() {
  portENTER_CRITICAL_ISR(&alsMux);
  if (!intlEvent) intlUs = micros();
  intlEvent = true;
  portEXIT_CRITICAL_ISR(&alsMux);
}

float secondsNow() { return millis() / 1000.0f; }

// ---- CMIS register access (paged) ----
bool readBytes(uint8_t dev, uint8_t offset, uint8_t *buf, size_t len);

bool writeByte(uint8_t dev, uint8_t offset, uint8_t value) {
  Wire.beginTransmission(dev);
  Wire.write(offset);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool cmisSelectPage(uint8_t page) {
  if (page == cmisPage) return true;
  if (!writeByte(cmis::ADDR, cmis::PAGE_SEL, page)) { cmisPage = 0xFF; return false; }
  cmisPage = page;
  return true;
}

// Reads lower memory (offset < 128) or the given upper page (offset >= 128); never crosses byte 127/128
bool cmisRead(uint8_t page, uint8_t offset, uint8_t *buf, size_t len) {
  if (offset >= 128 && !cmisSelectPage(page)) return false;
  return readBytes(cmis::ADDR, offset, buf, len);
}

bool cmisWrite(uint8_t page, uint8_t offset, uint8_t value) {
  if (offset >= 128 && !cmisSelectPage(page)) return false;
  return writeByte(cmis::ADDR, offset, value);
}

void setLaser(bool on) {
  if (moduleKind == MOD_CMIS) {
    digitalWrite(PIN_TX_DIS, LOW);
    cmisWrite(cmis::PAGE_LANE_CTRL, cmis::P10_TX_DISABLE, on ? 0x00 : 0x01);   // lane 1 output disable
  } else {
    digitalWrite(PIN_TX_DIS, on ? LOW : HIGH);
  }
}

// Reads (and so clears) all CMIS latched flags. Returns true if lane 1 Rx LOS was latched.
bool cmisServiceFlags() {
  uint8_t mod[6], lane[20];
  if (!cmisRead(0, cmis::MOD_FLAGS, mod, sizeof(mod))) return false;
  if (!cmisRead(cmis::PAGE_LANE_STATUS, cmis::P11_DP_CHANGED, lane, sizeof(lane))) return false;
  bool los = lane[cmis::P11_RX_LOS - cmis::P11_DP_CHANGED] & 0x01;
  if (los) { cmisLosSeenMs = millis(); cmisLosEver = true; }
  return los;
}

bool cmisLosRecent() { return cmisLosEver && millis() - cmisLosSeenMs < 1500; }

void reportShutdown(const char *how, long us) {
  outageCount++;
  probeCount = 0;
  alsTimerMs = millis();
  outageStartMs = alsTimerMs;
  if (us >= 0) Serial.printf("[%8.3fs] EVENT #%lu: %s -> laser OFF in %ld us\n", secondsNow(), outageCount, how, us);
  else         Serial.printf("[%8.3fs] EVENT #%lu: %s -> laser OFF\n", secondsNow(), outageCount, how);
}

// NORMAL -> SHUTDOWN -> PROBING -> NORMAL or SHUTDOWN
void runAls() {
  uint32_t now = millis();

  // Take a consistent snapshot of the data the ISRs can change
  portENTER_CRITICAL(&alsMux);
  bool event = losEvent;
  losEvent = false;
  bool intl = intlEvent;
  intlEvent = false;
  uint32_t tIntl = intlUs;
  AlsState state = alsState;
  uint32_t isrUs = isrEndUs - isrStartUs;
  portEXIT_CRITICAL(&alsMux);

  if (event) {
    // A different module on the bus (hot-swap) also moves this pin, so confirm the identifier first
    uint8_t id;
    if (readBytes(sff::ADDR_A0, 0, &id, 1) && id == moduleId) {
      reportShutdown("LOS interrupt, laser disabled inside ISR (TX_DISABLE pin)", (long)isrUs);
    } else {
      Serial.printf("[%8.3fs] LOS pin changed but the module changed too: ignoring, re-identifying\n", secondsNow());
      portENTER_CRITICAL(&alsMux);
      alsState = ALS_NORMAL;                             // undo the ISR's shutdown; the new module decides
      portEXIT_CRITICAL(&alsMux);
      moduleKind = MOD_NONE;                             // the next poll identifies the new module
      return;
    }
  }

  if (moduleKind == MOD_NONE) return;

  bool losNow;
  if (moduleKind == MOD_CMIS) {
    if (cmisHost != H_UP) return;
    // Service the flags on an IntL edge, or while IntL stays low (level), or on a timer
    static uint32_t lastService = 0;
    bool intlLow = digitalRead(PIN_LOS) == LOW;
    bool due = intl || (now - lastService >= (intlLow ? 50u : 200u)) ||
               (state == ALS_PROBING && now - lastService >= 50);
    bool losFlag = false;
    if (due) {
      lastService = now;
      losFlag = cmisServiceFlags();
      now = millis();          // I2C takes milliseconds: timestamps taken inside it are later than 'now'
    }
    if (state == ALS_NORMAL && losFlag) {
      setLaser(false);                                   // over I2C: page 10h byte 130 bit 0
      portENTER_CRITICAL(&alsMux);
      alsState = ALS_SHUTDOWN;
      portEXIT_CRITICAL(&alsMux);
      if (intl) reportShutdown("IntL interrupt -> Rx LOS flag -> laser disabled over I2C (page 10h)",
                               (long)(micros() - tIntl));
      else      reportShutdown("Rx LOS flag (IntL level / poll) -> laser disabled over I2C (page 10h)", -1);
      return;
    }
    // Light is back once no LOS was latched for 200 ms of the probe.
    // Signed difference: a timestamp newer than 'now' must never wrap into a huge unsigned age.
    uint32_t ref = (int32_t)(cmisLosSeenMs - alsTimerMs) > 0 ? cmisLosSeenMs : alsTimerMs;
    losNow = (int32_t)(now - ref) < 200;
  } else {
    losNow = digitalRead(PIN_LOS) == HIGH;
  }

  // Only the NORMAL state can be stale here, and NORMAL does nothing,
  // because the ISR acts only in NORMAL. So acting on the snapshot is safe.
  switch (state) {
    case ALS_NORMAL:
      break;

    case ALS_SHUTDOWN:
      if (now - alsTimerMs >= ALS_OFF_MS) {
        probeCount++;
        alsTimerMs = now;
        alsState = ALS_PROBING;
        setLaser(true);
        Serial.printf("[%8.3fs] ALS: restart probe #%lu, laser ON for %lu ms\n",
                      secondsNow(), probeCount, (unsigned long)ALS_PROBE_MS);
      }
      break;

    case ALS_PROBING:
      if (!losNow) {
        alsState = ALS_NORMAL;
        cmisLosEver = false;
        Serial.printf("[%8.3fs] ALS: light is back -> link RESTORED on probe #%lu, outage %.1f s\n",
                      secondsNow(), probeCount, (now - outageStartMs) / 1000.0f);
      } else if (now - alsTimerMs >= ALS_PROBE_MS) {
        setLaser(false);
        alsTimerMs = now;
        alsState = ALS_SHUTDOWN;
        Serial.printf("[%8.3fs] ALS: probe #%lu failed (still LOS), laser OFF again\n",
                      secondsNow(), probeCount);
      }
      break;
  }
}

// ================= Data link: bit-clock task =================

// ---- Bit-clock task state (touched only inside onBitClock) ----
uint8_t txPrbs = 0x7F, rxPrbs = 0;
uint8_t phase = 0, syncCount = 0;
bool locked = false;
float levelHi = 2000.0f, levelLo = 1000.0f;   // tracked '1' and '0' levels (ADC counts)
uint16_t winBits = 0, winErrors = 0;
uint8_t txFrame[frm::MAX_FRAME_BYTES];
int txFrameLen = 0, txFrameBit = 0;
uint16_t txSeq = 0;
frm::Receiver frameRx;

// Next bit of the outgoing frame stream; builds a new frame when the current one is finished
uint8_t nextFrameBit() {
  if (txFrameBit >= txFrameLen * 8) {
    char text[24];
    int len = snprintf(text, sizeof(text), "OPTIMON #%05u", (unsigned)txSeq);
    txFrameLen = frm::build(txFrame, txSeq, text, len);
    txSeq++;
    txFrameBit = 0;
    portENTER_CRITICAL(&linkMux);
    linkAcc.framesSent++;
    portEXIT_CRITICAL(&linkMux);
  }
  uint8_t b = (txFrame[txFrameBit >> 3] >> (7 - (txFrameBit & 7))) & 1;
  txFrameBit++;
  return b;
}

void resetLinkState() {
  txPrbs = 0x7F; rxPrbs = 0; syncCount = 0; locked = false;
  winBits = winErrors = 0;
  txFrameLen = txFrameBit = 0;
  txSeq = 0;
  frameRx.resetAll();
}

// Runs twice per bit: first half launches a bit, second half samples it in the middle
void onBitClock(void *) {
  if (phase == 0) {
    if (linkResetRequest) { resetLinkState(); linkResetRequest = false; }
    phase = 1;
    uint8_t bit = (linkMode == MODE_PRBS) ? prbs7Next(txPrbs) : nextFrameBit();
    digitalWrite(PIN_TX_DATA, bit);
    return;
  }
  phase = 0;

  // Decision: compare the ADC sample with a threshold halfway between the tracked levels
  int s = analogRead(PIN_RX_ADC);
  float thr = 0.5f * (levelHi + levelLo);
  uint8_t rx = (s > thr) ? 1 : 0;
  if (rx) { levelHi += (s - levelHi) * 0.05f;  levelLo += (s - levelLo) * 0.002f; }
  else    { levelLo += (s - levelLo) * 0.05f;  levelHi += (s - levelHi) * 0.002f; }

  if (!linkActive) { locked = false; syncCount = 0; frameRx.resync(); return; }

  // ---- Framed data: hunt for sync, collect the frame, check its CRC ----
  if (linkMode == MODE_FRAMES) {
    uint32_t missedBefore = frameRx.missed;
    frm::Receiver::Result r = frameRx.push(rx);
    if (r != frm::Receiver::NONE) {
      portENTER_CRITICAL(&linkMux);
      if (r == frm::Receiver::GOOD) {
        linkAcc.framesGood++;
        linkAcc.framesMissed += frameRx.missed - missedBefore;
        memcpy(linkAcc.lastText, frameRx.lastText, sizeof(linkAcc.lastText));
      } else {
        linkAcc.framesBad++;
      }
      portEXIT_CRITICAL(&linkMux);
    }
    return;
  }

  // ---- PRBS-7 BER test ----
  // Not locked: load 7 received bits into the checker's PRBS register
  if (!locked) {
    rxPrbs = (uint8_t)(((rxPrbs << 1) | rx) & 0x7F);
    if (++syncCount >= 7) {
      syncCount = 0;
      if (rxPrbs != 0) { locked = true; winBits = winErrors = 0; }   // all zeros = no signal
    }
    return;
  }

  // Locked: the checker runs its own PRBS and compares each received bit with it
  uint8_t expected = prbs7Next(rxPrbs);
  bool err = (rx != expected);

  bool lost = false;
  winBits++;
  if (err) winErrors++;
  if (winBits >= 256) {                       // more than 25% errors: we are out of sync
    lost = winErrors > 64;
    winBits = winErrors = 0;
  }

  portENTER_CRITICAL(&linkMux);
  linkAcc.bits++;
  if (err) linkAcc.errors++;
  if (expected) { linkAcc.n1++; linkAcc.sum1 += s; linkAcc.sq1 += (double)s * s; }
  else          { linkAcc.n0++; linkAcc.sum0 += s; linkAcc.sq0 += (double)s * s; }
  if (lost) linkAcc.lockLosses++;
  portEXIT_CRITICAL(&linkMux);

  if (lost) { locked = false; syncCount = 0; }
}

void clearLinkStats() {
  portENTER_CRITICAL(&linkMux);
  linkAcc = LinkStats();
  portEXIT_CRITICAL(&linkMux);
  linkLast = LinkResult();
}

void setBitRate(uint32_t rate) {
  bitRate = rate;
  esp_timer_stop(bitTimer);
  linkResetRequest = true;
  clearLinkStats();
  esp_timer_start_periodic(bitTimer, 1000000UL / bitRate / 2);
}

void startBitClock() {
  pinMode(PIN_TX_DATA, OUTPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  esp_timer_create_args_t args = {};
  args.callback = &onBitClock;
  args.name = "bitclk";
  esp_timer_create(&args, &bitTimer);
  esp_timer_start_periodic(bitTimer, 1000000UL / bitRate / 2);
}

void reportPrbs(const LinkStats &w) {
  LinkResult r;
  r.mode = MODE_PRBS;
  if (w.bits < 100 || w.n1 == 0 || w.n0 == 0) {
    linkLast = r;                              // invalid
    Serial.printf("[%8.3fs] BERT: NO LOCK (no usable signal%s)\n",
                  secondsNow(), linkActive ? "" : ", laser off");
    return;
  }

  double m1 = w.sum1 / w.n1, m0 = w.sum0 / w.n0;
  double s1 = sqrt(fmax(0.0, w.sq1 / w.n1 - m1 * m1));
  double s0 = sqrt(fmax(0.0, w.sq0 / w.n0 - m0 * m0));
  double q = (m1 - m0) / fmax(s1 + s0, 0.5);
  float estBer = (q > 0) ? 0.5f * erfcf((float)q / 1.41421356f) : 0.5f;

  r.valid = true;
  r.bits = w.bits;
  r.errors = w.errors;
  r.ber = (float)w.errors / w.bits;
  r.q = (float)q;
  linkLast = r;

  // With zero errors, BER is only known to be below 3/N at 95% confidence
  char berStr[24];
  if (w.errors == 0) snprintf(berStr, sizeof(berStr), "<%.1e (95%%)", 3.0 / w.bits);
  else               snprintf(berStr, sizeof(berStr), "%.1e", r.ber);

  Serial.printf("[%8.3fs] BERT @%lu bit/s: %lu bits, %lu errors -> BER %s | Q %.2f -> estimated BER %.1e"
                " | eye: '1' %.0f+/-%.1f mV, '0' %.0f+/-%.1f mV | lock losses %lu\n",
                secondsNow(), (unsigned long)bitRate, (unsigned long)w.bits, (unsigned long)w.errors, berStr,
                q, estBer, m1 * ADC_TO_MV, s1 * ADC_TO_MV, m0 * ADC_TO_MV, s0 * ADC_TO_MV,
                (unsigned long)w.lockLosses);
}

void reportFrames(const LinkStats &w) {
  LinkResult r;
  r.mode = MODE_FRAMES;
  uint32_t judged = w.framesGood + w.framesBad + w.framesMissed;
  if (w.framesGood == 0 && w.framesBad == 0) {
    linkLast = r;                              // invalid
    Serial.printf("[%8.3fs] FRAMES: NO SYNC (%lu sent%s)\n", secondsNow(), (unsigned long)w.framesSent,
                  linkActive ? "" : ", laser off");
    return;
  }

  uint32_t lost = w.framesBad + w.framesMissed;
  r.valid = true;
  r.frames = judged;
  r.frameErrors = lost;
  r.fer = (float)lost / judged;
  linkLast = r;

  // Frame error rate -> bit error rate, assuming independent errors over the checked bits
  const int checkedBits = 8 * (2 + frm::HEADER_BYTES + 14 + 2);   // sync + header + 14-char payload + CRC
  char ferStr[24], berStr[24];
  if (lost == 0) {
    snprintf(ferStr, sizeof(ferStr), "<%.1e (95%%)", 3.0 / judged);
    snprintf(berStr, sizeof(berStr), "<%.1e", 3.0 / judged / checkedBits);
  } else {
    snprintf(ferStr, sizeof(ferStr), "%.1e", r.fer);
    snprintf(berStr, sizeof(berStr), "%.1e", 1.0 - pow(1.0 - r.fer, 1.0 / checkedBits));
  }

  Serial.printf("[%8.3fs] FRAMES @%lu bit/s: sent %lu, good %lu, CRC errors %lu, missed %lu -> FER %s"
                " (implied BER %s) | last: \"%s\"\n",
                secondsNow(), (unsigned long)bitRate, (unsigned long)w.framesSent, (unsigned long)w.framesGood,
                (unsigned long)w.framesBad, (unsigned long)w.framesMissed, ferStr, berStr, w.lastText);
}

// Every BERT_WINDOW_MS: take the counts and print a report for the current mode
void updateLinkReport() {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < BERT_WINDOW_MS) return;
  lastMs = millis();

  portENTER_CRITICAL(&linkMux);
  LinkStats w = linkAcc;
  linkAcc = LinkStats();
  portEXIT_CRITICAL(&linkMux);

  if (linkMode == MODE_PRBS) reportPrbs(w);
  else                       reportFrames(w);
}

// ================= Serial commands =================
void printHelp() {
  Serial.println("Commands: prbs | frames | rate <bit/s> (100-5000) | help");
}

void runCommand(char *line) {
  while (*line == ' ') line++;
  if (strcmp(line, "prbs") == 0) {
    linkMode = MODE_PRBS;
    linkResetRequest = true;
    clearLinkStats();
    Serial.printf("[%8.3fs] MODE: %s\n", secondsNow(), MODE_NAMES[MODE_PRBS]);
  } else if (strcmp(line, "frames") == 0) {
    linkMode = MODE_FRAMES;
    linkResetRequest = true;
    clearLinkStats();
    Serial.printf("[%8.3fs] MODE: %s\n", secondsNow(), MODE_NAMES[MODE_FRAMES]);
  } else if (strncmp(line, "rate", 4) == 0) {
    long r = atol(line + 4);
    if (r < (long)MIN_BIT_RATE || r > (long)MAX_BIT_RATE) {
      Serial.printf("rate must be %lu-%lu bit/s\n", (unsigned long)MIN_BIT_RATE, (unsigned long)MAX_BIT_RATE);
      return;
    }
    setBitRate((uint32_t)r);
    Serial.printf("[%8.3fs] BIT RATE: %lu bit/s\n", secondsNow(), (unsigned long)bitRate);
  } else if (line[0] != '\0') {
    printHelp();
  }
}

// Collects characters into a line; runs it on Enter, or after 300 ms of silence
void handleSerial() {
  static char line[32];
  static uint8_t n = 0;
  static uint32_t lastCharMs = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    lastCharMs = millis();
    if (c == '\r') continue;
    if (c == '\n') { line[n] = '\0'; n = 0; runCommand(line); continue; }
    if (n < sizeof(line) - 1) line[n++] = (char)tolower(c);
  }
  if (n > 0 && millis() - lastCharMs > 300) { line[n] = '\0'; n = 0; runCommand(line); }
}

// ================= Module access and display =================
// Read 'len' bytes starting at 'offset', in chunks of up to 32 bytes
bool readBytes(uint8_t dev, uint8_t offset, uint8_t *buf, size_t len) {
  size_t done = 0;
  while (done < len) {
    size_t chunk = len - done;
    if (chunk > 32) chunk = 32;
    Wire.beginTransmission(dev);
    Wire.write((uint8_t)(offset + done));
    if (Wire.endTransmission(false) != 0) return false;   // repeated start
    if (Wire.requestFrom(dev, (uint8_t)chunk) != chunk) return false;
    for (size_t i = 0; i < chunk; i++) buf[done + i] = Wire.read();
    done += chunk;
  }
  return true;
}

void printThresholds() {
  Serial.println("Thresholds:       HighAlarm  LowAlarm  HighWarn   LowWarn");
  for (int p = 0; p < sff::NUM_PARAMS; p++)
    Serial.printf("  %-5s %-4s  %9.2f %9.2f %9.2f %9.2f\n", NAMES[p], UNITS[p],
                  limits[p].highAlarm, limits[p].lowAlarm, limits[p].highWarn, limits[p].lowWarn);
  Serial.println();
}

bool readSfpInfo() {
  uint8_t a0[96];
  if (!readBytes(sff::ADDR_A0, 0, a0, sizeof(a0))) { Serial.println("ERROR: no module at A0h (0x50)"); return false; }
  if (!sff::checksumOk(a0, 0, 62, a0[sff::A0_CC_BASE])) { Serial.println("ERROR: A0h checksum CC_BASE failed"); return false; }
  if (!(a0[sff::A0_DIAG_TYPE] & sff::DIAG_IMPLEMENTED)) { Serial.println("ERROR: module has no diagnostics"); return false; }

  sff::asciiField(vendor, a0 + sff::A0_VENDOR_NAME, 16);
  sff::asciiField(partNumber, a0 + sff::A0_VENDOR_PN, 16);
  wavelengthNm = sff::be16(a0 + sff::A0_WAVELENGTH);

  uint8_t a2[96];
  if (!readBytes(sff::ADDR_A2, 0, a2, sizeof(a2))) { Serial.println("ERROR: no diagnostics at A2h (0x51)"); return false; }
  if (!sff::checksumOk(a2, 0, 94, a2[sff::A2_CC_DMI])) { Serial.println("ERROR: A2h checksum CC_DMI failed"); return false; }
  for (int p = 0; p < sff::NUM_PARAMS; p++)
    limits[p] = sff::decodeLimits(a2 + sff::A2_THRESHOLDS, (sff::Param)p);

  Serial.printf("\nModule: SFP (SFF-8472) | %s | %s | %u nm | checksums OK\n", vendor, partNumber, wavelengthNm);
  printThresholds();
  return true;
}

bool readCmisInfo() {
  cmisPage = 0xFF;
  uint8_t lo[4];
  if (!cmisRead(0, 0, lo, sizeof(lo))) { Serial.println("ERROR: CMIS lower page read failed"); return false; }
  if (lo[cmis::MEM_MODEL] & cmis::MEM_FLAT) { Serial.println("ERROR: flat-memory CMIS module not supported"); return false; }

  if (!writeByte(cmis::ADDR, cmis::BANK_SEL, 0)) return false;          // bank 0
  uint8_t p0[96];                                                       // page 00h bytes 128..223
  if (!cmisRead(cmis::PAGE_ADMIN, 128, p0, sizeof(p0))) { Serial.println("ERROR: page 00h read failed"); return false; }
  if (!sff::checksumOk(p0, 0, 93, p0[cmis::P00_CHECKSUM - 128])) { Serial.println("ERROR: page 00h checksum failed"); return false; }
  sff::asciiField(vendor, p0 + (cmis::P00_VENDOR_NAME - 128), 16);
  sff::asciiField(partNumber, p0 + (cmis::P00_VENDOR_PN - 128), 16);
  wavelengthNm = 0;

  uint8_t p2[72];                                                       // page 02h bytes 128..199
  if (!cmisRead(cmis::PAGE_THRESHOLDS, 128, p2, sizeof(p2))) { Serial.println("ERROR: page 02h read failed"); return false; }
  for (int p = 0; p < sff::NUM_PARAMS; p++) limits[p] = cmis::limitsFromPage02(p2, (sff::Param)p);

  cmisState = cmis::moduleState(lo[cmis::STATE]);
  Serial.printf("\nModule: QSFP-DD (CMIS %u.%u) | %s | %s | page 00h checksum OK | state %s\n",
                lo[cmis::REV] >> 4, lo[cmis::REV] & 0x0F, vendor, partNumber, cmis::moduleStateName(cmisState));
  printThresholds();
  return true;
}

bool interruptAttached = false;

void attachModuleInterrupt() {
  if (interruptAttached) detachInterrupt(digitalPinToInterrupt(PIN_LOS));
  interruptAttached = false;
  portENTER_CRITICAL(&alsMux);
  alsState = ALS_NORMAL;
  losEvent = intlEvent = false;
  portEXIT_CRITICAL(&alsMux);
  if (moduleKind == MOD_SFP) {
    attachInterrupt(digitalPinToInterrupt(PIN_LOS), onLosRising, RISING);
    interruptAttached = true;
    if (digitalRead(PIN_LOS) == HIGH) onLosRising();   // already dark: no edge will come
  } else if (moduleKind == MOD_CMIS) {
    attachInterrupt(digitalPinToInterrupt(PIN_LOS), onIntL, FALLING);
    interruptAttached = true;
  }
}

// Reads the SFF-8024 identifier at 0x50 byte 0 and sets up the matching driver
bool identifyModule() {
  uint8_t id;
  moduleKind = MOD_NONE;
  cmisHost = H_IDLE;
  if (!readBytes(sff::ADDR_A0, 0, &id, 1)) { Serial.println("ERROR: no module at 0x50"); return false; }
  moduleId = id;
  if (id == cmis::ID_SFP) {
    if (!readSfpInfo()) return false;
    moduleKind = MOD_SFP;
    setLaser(true);
  } else if (cmis::isCmisId(id)) {
    if (!readCmisInfo()) return false;
    moduleKind = MOD_CMIS;
    digitalWrite(PIN_TX_DIS, LOW);
    cmisHost = H_RELEASE_LOWPWR;
    cmisBringupMs = millis();
  } else {
    Serial.printf("ERROR: unsupported module identifier 0x%02X\n", id);
    return false;
  }
  attachModuleInterrupt();
  return true;
}

// Host side of CMIS bring-up, one step every CMIS_STEP_MS
void runCmisHost() {
  if (moduleKind != MOD_CMIS || cmisHost == H_UP || cmisHost == H_FAILED) return;
  uint32_t now = millis();
  if (now - cmisStepMs < CMIS_STEP_MS) return;
  cmisStepMs = now;
  float t = (now - cmisBringupMs) / 1000.0f;

  switch (cmisHost) {
    case H_RELEASE_LOWPWR: {
      uint8_t ctrl;
      if (!cmisRead(0, cmis::MOD_CTRL, &ctrl, 1)) return;
      cmisWrite(0, cmis::MOD_CTRL, ctrl & ~cmis::CTRL_LOWPWR_SW);
      Serial.printf("[%8.3fs] CMIS +%.2fs: host clears LowPwrRequestSW (byte 26 bit 4)\n", secondsNow(), t);
      cmisHost = H_WAIT_READY;
      cmisPhaseMs = now;
      break;
    }
    case H_WAIT_READY: {
      uint8_t b3;
      if (!cmisRead(0, cmis::STATE, &b3, 1)) return;
      cmis::ModuleState s = cmis::moduleState(b3);
      if (s != cmisState) {
        Serial.printf("[%8.3fs] CMIS +%.2fs: %s -> %s\n", secondsNow(), t,
                      cmis::moduleStateName(cmisState), cmis::moduleStateName(s));
        cmisState = s;
      }
      if (s == cmis::MS_READY) { cmisHost = H_RELEASE_DP; cmisPhaseMs = now; }
      else if (now - cmisPhaseMs > CMIS_TIMEOUT_MS) {
        Serial.printf("[%8.3fs] CMIS: timeout waiting for ModuleReady (state %s)\n", secondsNow(), cmis::moduleStateName(s));
        cmisHost = H_FAILED;
      }
      break;
    }
    case H_RELEASE_DP:
      cmisWrite(cmis::PAGE_LANE_CTRL, cmis::P10_TX_DISABLE, 0x00);       // lane 1 output enabled
      cmisWrite(cmis::PAGE_LANE_CTRL, cmis::P10_DP_DEINIT, 0x00);        // release lane 1 data path
      Serial.printf("[%8.3fs] CMIS +%.2fs: host clears DPDeinit lane 1 (page 10h byte 128)\n", secondsNow(), t);
      cmisHost = H_WAIT_DP;
      cmisPhaseMs = now;
      break;
    case H_WAIT_DP: {
      uint8_t dp[4];
      if (!cmisRead(cmis::PAGE_LANE_STATUS, cmis::P11_DP_STATE, dp, sizeof(dp))) return;
      cmis::DataPathState s = cmis::dpState(dp, 1);
      if (s != cmisDp) {
        Serial.printf("[%8.3fs] CMIS +%.2fs: lane 1 %s -> %s\n", secondsNow(), t,
                      cmis::dpStateName(cmisDp), cmis::dpStateName(s));
        cmisDp = s;
      }
      if (s == cmis::DP_ACTIVATED) {
        cmisServiceFlags();                                              // clear bring-up flags
        cmisHost = H_UP;
        Serial.printf("[%8.3fs] CMIS +%.2fs: link UP, lane 1 carrying traffic\n\n", secondsNow(), t);
      } else if (now - cmisPhaseMs > CMIS_TIMEOUT_MS) {
        Serial.printf("[%8.3fs] CMIS: timeout waiting for DPActivated (state %s)\n", secondsNow(), cmis::dpStateName(s));
        cmisHost = H_FAILED;
      }
      break;
    }
    default:
      break;
  }
}

// One diagnostics read. Fills values, LOS and laser state; returns false on an I2C error.
bool readDiagnostics(float *value, bool *los, bool *txOff) {
  if (moduleKind == MOD_SFP) {
    uint8_t buf[16];                               // A2h bytes 96..111
    if (!readBytes(sff::ADDR_A2, sff::A2_MONITORS, buf, sizeof(buf))) return false;
    for (int p = 0; p < sff::NUM_PARAMS; p++) value[p] = sff::decodeMonitor(buf, (sff::Param)p);
    uint8_t status = buf[sff::A2_STATUS - sff::A2_MONITORS];
    *los = status & sff::STATUS_RX_LOS;
    *txOff = status & sff::STATUS_TX_DISABLE;
    return true;
  }
  // CMIS: module monitors in lower memory, lane monitors in page 11h (flags are left to cmisServiceFlags)
  uint8_t lo[4], st[4], mon[34];
  if (!cmisRead(0, cmis::TEMP, lo, sizeof(lo))) return false;
  if (!cmisRead(cmis::PAGE_LANE_STATUS, cmis::P11_DP_STATE, st, sizeof(st))) return false;
  if (!cmisRead(cmis::PAGE_LANE_STATUS, cmis::P11_TXPWR, mon, sizeof(mon))) return false;
  uint8_t b3;
  if (!cmisRead(0, cmis::STATE, &b3, 1)) return false;
  cmisState = cmis::moduleState(b3);
  cmisDp = cmis::dpState(st, 1);
  value[sff::P_TEMP] = sff::decode(sff::P_TEMP, sff::be16(lo));
  value[sff::P_VCC]  = sff::decode(sff::P_VCC, sff::be16(lo + 2));
  value[sff::P_TX]   = sff::decode(sff::P_TX, sff::be16(mon));                                    // byte 154
  value[sff::P_BIAS] = sff::decode(sff::P_BIAS, sff::be16(mon + (cmis::P11_TXBIAS - cmis::P11_TXPWR)));
  value[sff::P_RX]   = sff::decode(sff::P_RX, sff::be16(mon + (cmis::P11_RXPWR - cmis::P11_TXPWR)));
  *los = cmisHost == H_UP && cmisLosRecent();
  *txOff = cmisDp != cmis::DP_ACTIVATED || alsState == ALS_SHUTDOWN;
  return true;
}

void updateLeds(sff::Level worst, bool los) {
  static bool blink = false;
  blink = !blink;
  bool healthy = (worst == sff::L_OK) && !los;
  digitalWrite(PIN_LED_GREEN, healthy ? HIGH : LOW);
  if (los || sff::isAlarm(worst))  digitalWrite(PIN_LED_RED, HIGH);     // alarm: solid red
  else if (worst != sff::L_OK)     digitalWrite(PIN_LED_RED, blink);    // warning: blinking red
  else                             digitalWrite(PIN_LED_RED, LOW);
}

void drawOled(const float *value, const sff::Level *level, sff::Level worst, bool los) {
  const char *link = los ? "LINK: LOS!" : sff::isAlarm(worst) ? "LINK: ALARM"
                   : (worst != sff::L_OK) ? "LINK: WARN" : "LINK: UP";
  if (moduleKind == MOD_CMIS && cmisHost != H_UP) link = "LINK: INIT";
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  if (moduleKind == MOD_CMIS)
    oled.printf("%-12s %s\n", partNumber, cmisDp == cmis::DP_ACTIVATED ? "ACT" : cmis::moduleStateShort(cmisState));
  else
    oled.printf("%-12s %4unm\n", partNumber, wavelengthNm);
  oled.printf("%-10s ALS:%s\n", link, ALS_NAMES[alsState]);
  oled.drawFastHLine(0, 18, 128, SSD1306_WHITE);
  oled.setCursor(0, 22);
  for (int p = 0; p < sff::NUM_PARAMS; p++) {
    if (p == sff::P_VCC) continue;                          // make room for the link line
    oled.printf("%-5s%7.*f%-4s %s\n", NAMES[p], DECIMALS[p], value[p], UNITS[p], sff::levelName(level[p]));
  }
  if (linkMode == MODE_FRAMES) {
    if (!linkLast.valid || linkLast.mode != MODE_FRAMES) oled.println("FRAMES  NO SYNC");
    else if (linkLast.frameErrors == 0) oled.printf("FER 0/%lu frames\n", (unsigned long)linkLast.frames);
    else                                oled.printf("FER %.1e %lufr\n", linkLast.fer, (unsigned long)linkLast.frames);
  } else {
    float q = fminf(linkLast.q, 99.9f);
    if (!linkLast.valid || linkLast.mode != MODE_PRBS) oled.println("BER  NO LOCK");
    else if (linkLast.errors == 0) oled.printf("BER <%.0e Q%5.1f\n", 3.0 / linkLast.bits, q);
    else                           oled.printf("BER %.1e Q%5.1f\n", linkLast.ber, q);
  }
  oled.display();
}

void showNoModule() {
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, HIGH);
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("NO MODULE");
  oled.println("check I2C / EEPROM");
  oled.display();
}

// ================= Setup and main loop =================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_TX_DIS, OUTPUT);
  pinMode(PIN_LOS, INPUT);
  digitalWrite(PIN_TX_DIS, LOW);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);                         // fast-mode I2C
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("WARNING: OLED not found");
  oled.clearDisplay();
  oled.display();

  Serial.println("\nOptiMon - optical module manager (SFP / SFF-8472 and QSFP-DD / CMIS)");
  delay(50);                                     // let the module settle
  identifyModule();

  startBitClock();
  Serial.printf("LINK: %s at %lu bit/s, sampling mid-bit, %lu s windows\n",
                MODE_NAMES[linkMode], (unsigned long)bitRate, (unsigned long)(BERT_WINDOW_MS / 1000));
  printHelp();
  Serial.println();
}

void loop() {
  runCmisHost();
  runAls();                                      // runs on every pass, not once a second
  linkActive = (alsState == ALS_NORMAL) && (moduleKind == MOD_SFP || cmisHost == H_UP);
  handleSerial();
  updateLinkReport();

  static uint32_t lastPoll = 0, lastPrint = 0;
  if (millis() - lastPoll < POLL_MS) return;
  lastPoll = millis();

  // Hot-swap check: a different identifier at 0x50 byte 0 means a different module
  uint8_t id;
  bool present = readBytes(sff::ADDR_A0, 0, &id, 1);
  if (moduleKind == MOD_NONE || !present || id != moduleId) {
    if (moduleKind != MOD_NONE)
      Serial.printf("[%8.3fs] MODULE CHANGED (identifier 0x%02X -> 0x%02X), re-identifying\n",
                    secondsNow(), moduleId, present ? id : 0);
    if (!identifyModule()) { showNoModule(); return; }
  }

  float value[sff::NUM_PARAMS];
  bool los = false, txOff = false;
  if (!readDiagnostics(value, &los, &txOff)) {
    Serial.println("ERROR: diagnostics read failed");
    moduleKind = MOD_NONE;
    return;
  }

  sff::Level level[sff::NUM_PARAMS];
  sff::Level worst = sff::L_OK;
  for (int p = 0; p < sff::NUM_PARAMS; p++) {
    level[p] = sff::classify(value[p], limits[p]);
    if (moduleKind == MOD_CMIS && cmisHost != H_UP && p >= sff::P_BIAS) level[p] = sff::L_OK;  // lane not live yet
    if (level[p] > worst) worst = level[p];
  }

  if (millis() - lastPrint >= DIAG_PRINT_MS) {
    lastPrint = millis();
    Serial.printf("[%5lus]", (unsigned long)(millis() / 1000));
    for (int p = 0; p < sff::NUM_PARAMS; p++)
      Serial.printf(" %s %.*f%s %s |", NAMES[p], DECIMALS[p], value[p], UNITS[p], sff::levelName(level[p]));
    Serial.printf(" LOS=%d TXDIS=%d ALS=%s", los, txOff, ALS_NAMES[alsState]);
    if (moduleKind == MOD_CMIS) Serial.printf(" | %s %s", cmis::moduleStateName(cmisState), cmis::dpStateName(cmisDp));
    Serial.println();
  }

  updateLeds(worst, los);
  drawOled(value, level, worst, los);
}
