// OptiMon - SFP monitor + Automatic Laser Shutdown + PRBS-7 bit error rate tester (BERT)
#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_timer.h"
#include "sff8472.h"
#include "prbs7.h"

const int PIN_SDA = 21, PIN_SCL = 22;
const int PIN_LED_GREEN = 26, PIN_LED_RED = 27;
const int PIN_TX_DIS = 33;              // HIGH = laser off
const int PIN_LOS = 35;                 // from the module, HIGH = loss of signal
const int PIN_TX_DATA = 25;             // bits to the laser
const int PIN_RX_ADC = 34;              // analog receiver output from the module
const uint32_t POLL_MS = 1000;          // diagnostics poll period
const uint32_t DIAG_PRINT_MS = 5000;    // how often diagnostics are printed
const uint32_t ALS_OFF_MS = 5000;       // laser stays off this long before a restart probe
const uint32_t ALS_PROBE_MS = 2000;     // laser on for this long to test the fiber
const uint32_t BIT_RATE = 1000;         // bits per second
const uint32_t HALF_BIT_US = 1000000UL / BIT_RATE / 2;
const uint32_t BERT_WINDOW_MS = 5000;   // BER measurement window
const float ADC_TO_MV = 3300.0f / 4095.0f;

enum AlsState : uint8_t { ALS_NORMAL, ALS_SHUTDOWN, ALS_PROBING };
const char *ALS_NAMES[] = {"ON", "OFF", "PROBE"};

Adafruit_SSD1306 oled(128, 64, &Wire, -1);

const char *NAMES[sff::NUM_PARAMS] = {"Temp", "Vcc", "Bias", "TxPwr", "RxPwr"};
const char *UNITS[sff::NUM_PARAMS] = {" C", " V", " mA", " dBm", " dBm"};
const int   DECIMALS[sff::NUM_PARAMS] = {1, 2, 1, 2, 2};

sff::Limits limits[sff::NUM_PARAMS];
char vendor[17], partNumber[17];
uint16_t wavelengthNm = 0;
bool moduleOk = false;

// ================= Automatic Laser Shutdown =================
// ---- Shared between the LOS interrupt handler and the main loop ----
volatile AlsState alsState = ALS_NORMAL;
volatile bool losEvent = false;
volatile uint32_t isrStartUs = 0, isrEndUs = 0;
portMUX_TYPE alsMux = portMUX_INITIALIZER_UNLOCKED;   // protects data shared with the ISR

// ---- Main-loop-only ALS bookkeeping ----
uint32_t alsTimerMs = 0, outageStartMs = 0;
unsigned long probeCount = 0, outageCount = 0;

// LOS rising edge: shut the laser off right here in the ISR (eye safety must not wait for the main loop)
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

void setLaser(bool on) { digitalWrite(PIN_TX_DIS, on ? LOW : HIGH); }

float secondsNow() { return millis() / 1000.0f; }

// NORMAL -> SHUTDOWN -> PROBING -> NORMAL or SHUTDOWN
void runAls() {
  uint32_t now = millis();

  // Take a consistent snapshot of the data the ISR can change
  portENTER_CRITICAL(&alsMux);
  bool event = losEvent;
  losEvent = false;
  AlsState state = alsState;
  uint32_t isrUs = isrEndUs - isrStartUs;
  portEXIT_CRITICAL(&alsMux);

  if (event) {                          // report what the ISR already did
    outageCount++;
    probeCount = 0;
    alsTimerMs = now;
    outageStartMs = now;
    Serial.printf("[%8.3fs] EVENT #%lu: LOS interrupt -> laser OFF in %lu us (inside ISR)\n",
                  secondsNow(), outageCount, (unsigned long)isrUs);
  }

  bool losNow = digitalRead(PIN_LOS) == HIGH;

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

// ================= Bit error rate tester (BERT) =================
struct BertStats {
  uint32_t bits = 0, errors = 0, lockLosses = 0;
  uint32_t n1 = 0, n0 = 0;                     // samples of known '1's and '0's
  double sum1 = 0, sq1 = 0, sum0 = 0, sq0 = 0; // for mean and spread of each level
};
BertStats bertAcc;                             // filled by the bit-clock task
portMUX_TYPE bertMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool bertActive = true;               // false while ALS has the laser off

struct BertResult {
  bool valid = false;
  uint32_t bits = 0, errors = 0;
  float ber = 0, q = 0, estBer = 0;
};
BertResult bertLast;                           // main loop only

// ---- Bit-clock task state (touched only inside onBitClock) ----
uint8_t txPrbs = 0x7F, rxPrbs = 0;
uint8_t phase = 0, syncCount = 0;
bool locked = false;
float levelHi = 2000.0f, levelLo = 1000.0f;   // tracked '1' and '0' levels (ADC counts)
uint16_t winBits = 0, winErrors = 0;


// Runs twice per bit: first half launches a bit, second half samples it in the middle
void onBitClock(void *) {
  if (phase == 0) {
    phase = 1;
    digitalWrite(PIN_TX_DATA, prbs7Next(txPrbs));
    return;
  }
  phase = 0;

  // Decision: compare the ADC sample with a threshold halfway between the tracked levels
  int s = analogRead(PIN_RX_ADC);
  float thr = 0.5f * (levelHi + levelLo);
  uint8_t rx = (s > thr) ? 1 : 0;
  if (rx) { levelHi += (s - levelHi) * 0.05f;  levelLo += (s - levelLo) * 0.002f; }
  else    { levelLo += (s - levelLo) * 0.05f;  levelHi += (s - levelHi) * 0.002f; }

  if (!bertActive) { locked = false; syncCount = 0; return; }

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

  portENTER_CRITICAL(&bertMux);
  bertAcc.bits++;
  if (err) bertAcc.errors++;
  if (expected) { bertAcc.n1++; bertAcc.sum1 += s; bertAcc.sq1 += (double)s * s; }
  else          { bertAcc.n0++; bertAcc.sum0 += s; bertAcc.sq0 += (double)s * s; }
  if (lost) bertAcc.lockLosses++;
  portEXIT_CRITICAL(&bertMux);

  if (lost) { locked = false; syncCount = 0; }
}

void startBitClock() {
  pinMode(PIN_TX_DATA, OUTPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  esp_timer_create_args_t args = {};
  args.callback = &onBitClock;
  args.name = "bitclk";
  esp_timer_handle_t bitTimer;
  esp_timer_create(&args, &bitTimer);
  esp_timer_start_periodic(bitTimer, HALF_BIT_US);
}

// Every BERT_WINDOW_MS: take the counts, compute BER and Q-factor, print a report
void updateBert() {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < BERT_WINDOW_MS) return;
  lastMs = millis();

  portENTER_CRITICAL(&bertMux);
  BertStats w = bertAcc;
  bertAcc = BertStats();
  portEXIT_CRITICAL(&bertMux);

  BertResult r;
  if (w.bits < 100 || w.n1 == 0 || w.n0 == 0) {
    bertLast = r;                              // invalid
    Serial.printf("[%8.3fs] BERT: NO LOCK (no usable signal%s)\n",
                  secondsNow(), bertActive ? "" : ", laser off by ALS");
    return;
  }

  double m1 = w.sum1 / w.n1, m0 = w.sum0 / w.n0;
  double s1 = sqrt(fmax(0.0, w.sq1 / w.n1 - m1 * m1));
  double s0 = sqrt(fmax(0.0, w.sq0 / w.n0 - m0 * m0));
  double q = (m1 - m0) / fmax(s1 + s0, 0.5);

  r.valid = true;
  r.bits = w.bits;
  r.errors = w.errors;
  r.ber = (float)w.errors / w.bits;
  r.q = (float)q;
  r.estBer = (q > 0) ? 0.5f * erfcf((float)q / 1.41421356f) : 0.5f;
  bertLast = r;

  char berStr[16];
  if (w.errors == 0) snprintf(berStr, sizeof(berStr), "<%.0e", 1.0 / w.bits);
  else               snprintf(berStr, sizeof(berStr), "%.1e", r.ber);

  Serial.printf("[%8.3fs] BERT %lu bits, %lu errors -> BER %s | Q %.2f -> estimated BER %.1e"
                " | eye: '1' %.0f+/-%.1f mV, '0' %.0f+/-%.1f mV | lock losses %lu\n",
                secondsNow(), (unsigned long)w.bits, (unsigned long)w.errors, berStr,
                q, r.estBer, m1 * ADC_TO_MV, s1 * ADC_TO_MV, m0 * ADC_TO_MV, s0 * ADC_TO_MV,
                (unsigned long)w.lockLosses);
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

bool readModuleInfo() {
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

  Serial.printf("\nModule: %s | %s | %u nm | checksums OK\n", vendor, partNumber, wavelengthNm);
  Serial.println("Thresholds:       HighAlarm  LowAlarm  HighWarn   LowWarn");
  for (int p = 0; p < sff::NUM_PARAMS; p++)
    Serial.printf("  %-5s %-4s  %9.2f %9.2f %9.2f %9.2f\n", NAMES[p], UNITS[p],
                  limits[p].highAlarm, limits[p].lowAlarm, limits[p].highWarn, limits[p].lowWarn);
  Serial.println();
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
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.printf("%-12s %4unm\n", partNumber, wavelengthNm);
  oled.printf("%-10s ALS:%s\n", link, ALS_NAMES[alsState]);
  oled.drawFastHLine(0, 18, 128, SSD1306_WHITE);
  oled.setCursor(0, 22);
  for (int p = 0; p < sff::NUM_PARAMS; p++) {
    if (p == sff::P_VCC) continue;                          // make room for the BER line
    oled.printf("%-5s%7.*f%-4s %s\n", NAMES[p], DECIMALS[p], value[p], UNITS[p], sff::levelName(level[p]));
  }
  float q = fminf(bertLast.q, 99.9f);
  if (!bertLast.valid)           oled.println("BER  NO LOCK");
  else if (bertLast.errors == 0) oled.printf("BER <%.0e Q%5.1f\n", 1.0 / bertLast.bits, q);
  else                           oled.printf("BER %.1e Q%5.1f\n", bertLast.ber, q);
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
  setLaser(true);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);                         // fast-mode I2C
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("WARNING: OLED not found");
  oled.clearDisplay();
  oled.display();

  Serial.println("\nOptiMon - SFP monitor + Automatic Laser Shutdown + PRBS-7 BER tester");
  moduleOk = readModuleInfo();

  attachInterrupt(digitalPinToInterrupt(PIN_LOS), onLosRising, RISING);
  delay(50);                                     // let the module settle
  if (digitalRead(PIN_LOS) == HIGH) {            // already dark at boot: no edge will come
    onLosRising();                               // it takes the lock itself
  }

  startBitClock();
  Serial.printf("BERT: PRBS-7 at %lu bit/s, sampling mid-bit, %lu s windows\n\n",
                (unsigned long)BIT_RATE, (unsigned long)(BERT_WINDOW_MS / 1000));
}

void loop() {
  runAls();                                      // runs on every pass, not once a second
  bertActive = (alsState == ALS_NORMAL);
  updateBert();

  static uint32_t lastPoll = 0, lastPrint = 0;
  if (millis() - lastPoll < POLL_MS) return;
  lastPoll = millis();

  if (!moduleOk) {
    moduleOk = readModuleInfo();
    if (!moduleOk) { showNoModule(); return; }
  }

  uint8_t buf[16];                               // A2h bytes 96..111
  if (!readBytes(sff::ADDR_A2, sff::A2_MONITORS, buf, sizeof(buf))) {
    Serial.println("ERROR: diagnostics read failed");
    moduleOk = false;
    return;
  }

  float value[sff::NUM_PARAMS];
  sff::Level level[sff::NUM_PARAMS];
  sff::Level worst = sff::L_OK;
  for (int p = 0; p < sff::NUM_PARAMS; p++) {
    value[p] = sff::decodeMonitor(buf, (sff::Param)p);
    level[p] = sff::classify(value[p], limits[p]);
    if (level[p] > worst) worst = level[p];
  }
  uint8_t status = buf[sff::A2_STATUS - sff::A2_MONITORS];
  bool los   = status & sff::STATUS_RX_LOS;
  bool txOff = status & sff::STATUS_TX_DISABLE;

  if (millis() - lastPrint >= DIAG_PRINT_MS) {
    lastPrint = millis();
    Serial.printf("[%5lus]", (unsigned long)(millis() / 1000));
    for (int p = 0; p < sff::NUM_PARAMS; p++)
      Serial.printf(" %s %.*f%s %s |", NAMES[p], DECIMALS[p], value[p], UNITS[p], sff::levelName(level[p]));
    Serial.printf(" LOS=%d TXDIS=%d ALS=%s\n", los, txOff, ALS_NAMES[alsState]);
  }

  updateLeds(worst, los);
  drawOled(value, level, worst, los);
}
//