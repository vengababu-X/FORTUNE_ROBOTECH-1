/*
 * Smart Tapper v2 - prototype controller
 * Board : ESP32 DevKit (ESP32-WROOM-32, classic ESP32 - needed for BluetoothSerial)
 * Core  : Arduino-ESP32 core 3.x
 * Libraries (Arduino Library Manager):
 *   - "HX711" by Bogdan Necula
 *   - "Adafruit PN532" by Adafruit
 *   - "RTClib" by Adafruit
 *
 * ALL values marked CALIBRATE are starting guesses. Find the real values
 * with the tests in Section 10 of the project document before using on trees.
 */
#include <Wire.h>
#include <HX711.h>
#include <Adafruit_PN532.h>
#include <RTClib.h>
#include <BluetoothSerial.h>

// ---------------- Pins ----------------
const int CAR_RPWM = 25, CAR_LPWM = 26;       // BTS7960: RPWM = carriage DOWN the rail (cutting)
const int ENC_A = 32, ENC_B = 33;             // carriage motor hall encoder
const int DEPTH_STEP = 14, DEPTH_DIR = 27;    // TMC2209 #1 - blade depth
const int PROBE_STEP = 13, PROBE_DIR = 4;     // TMC2209 #2 - needle probe
const int STEPPER_EN = 23;                    // LOW = drivers enabled
const int LIM_TOP = 18, LIM_BOTTOM = 19;      // adjustable end stops on rail (to GND)
const int BTN_START = 16;                     // start button (to GND)
const int BTN_ESTOP = 35;                     // E-stop aux contact to GND; input-only pin: add external 10k pull-up to 3.3 V
const int LED = 2, BUZZER = 17;
const int NFC_IRQ = 34, NFC_RESET = 15;       // PN532 IRQ / RSTPDN
const int CURRENT_PIN = 36;                   // ACS712-20A via 10k/20k divider (5 V -> 3.3 V)
const int HX_DOUT = 39, HX_SCK = 5;           // HX711 - set its RATE pin HIGH (80 samples/s)
// I2C (PN532 in I2C mode + DS3231): SDA = 21, SCL = 22

// ---------------- Mechanics ----------------
const float STEPS_PER_MM = 800.0;       // 200 step motor, 1/8 microstep, Tr8x2 lead screw
const float RESIDUAL_BARK_MM = 1.0;     // c: bark left over the cambium
const float MAX_PROBE_MM = 16.0;
const float MAX_DEPTH_MM = 12.0;
const float MIN_VALID_T = 3.0, MAX_VALID_T = 15.0;  // plausible bark thickness range (mm)
const long  PROBE_OFFSET_COUNTS = 300;  // probe is this many encoder counts AHEAD of blade (CALIBRATE)
const int   CUT_PWM = 200, MOVE_PWM = 170, SLOW_PWM = 110;
const float CURRENT_LIMIT_A = 10.0;     // CALIBRATE after measuring normal cutting current
const int   RECORD_MAX_AGE_DAYS = 30;   // re-gauge bark once a month

// ---------------- Probe thresholds (CALIBRATE on real bark) ----------------
const float PROBE_STEP_MM = 0.1;
const float CONTACT_G = 50.0;           // force at first touch of bark
const float WOOD_RISE_G = 600.0;        // force rise over last 0.3 mm that means "wood reached"

// ---------------- Objects / state ----------------
HX711 scale;
Adafruit_PN532 nfc(NFC_IRQ, NFC_RESET);  // I2C mode
RTC_DS3231 rtc;
BluetoothSerial bt;

volatile long encCount = 0;
long railCounts = 0;                    // encoder counts from top stop to bottom stop
long depthPosSteps = 0;                 // 0 = blade tip flush with skid roller
float barkT[3];                         // bark thickness at 5 %, 50 %, 95 % of rail
const float GAUGE_FRAC[3] = {0.05, 0.50, 0.95};

uint8_t tagUid[7]; uint8_t tagUidLen = 0; bool tagPresent = false;

void IRAM_ATTR onEncoder() { encCount += (digitalRead(ENC_B) ? -1 : 1); }

// ---------------- Low-level helpers ----------------
void logMsg(const String &s) { Serial.println(s); bt.println(s); }
void beep(int n) { for (int i = 0; i < n; i++) { digitalWrite(BUZZER, HIGH); delay(120); digitalWrite(BUZZER, LOW); delay(120); } }
bool estop() { return digitalRead(BTN_ESTOP) == LOW; }

float motorAmps() {
  float mv = analogReadMilliVolts(CURRENT_PIN) / 0.667;   // undo divider
  return fabs((mv - 2500.0) / 100.0);                     // ACS712-20A: 100 mV/A, 2.5 V at 0 A
}

void carriage(int speed) {             // +down, -up, 0 stop
  if (speed > 0)      { analogWrite(CAR_RPWM, speed); analogWrite(CAR_LPWM, 0); }
  else if (speed < 0) { analogWrite(CAR_RPWM, 0);     analogWrite(CAR_LPWM, -speed); }
  else                { analogWrite(CAR_RPWM, 0);     analogWrite(CAR_LPWM, 0); }
}

void stepMotor(int stepPin, int dirPin, bool forward, long steps, int usHalf = 60) {
  digitalWrite(dirPin, forward ? HIGH : LOW);
  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH); delayMicroseconds(usHalf);
    digitalWrite(stepPin, LOW);  delayMicroseconds(usHalf);
  }
}

void setBladeDepth(float mm) {
  mm = constrain(mm, 0.0f, MAX_DEPTH_MM);
  long target = lround(mm * STEPS_PER_MM);
  long diff = target - depthPosSteps;
  if (diff) stepMotor(DEPTH_STEP, DEPTH_DIR, diff > 0, labs(diff));
  depthPosSteps = target;
}

long probePosSteps = 0;
void retractProbe() { if (probePosSteps) stepMotor(PROBE_STEP, PROBE_DIR, false, probePosSteps); probePosSteps = 0; }

void fault(const char *msg) {
  carriage(0);
  retractProbe();
  setBladeDepth(0);
  logMsg(String("FAULT: ") + msg);
  digitalWrite(LED, LOW);
  while (true) { beep(3); delay(1500); }     // stay safe until power-cycled
}

// Move carriage so that encoder reaches target (counts from top stop)
void moveTo(long target) {
  unsigned long t0 = millis();
  while (labs(target - encCount) > 5) {
    if (estop()) fault("E-stop");
    long err = target - encCount;
    int pwm = labs(err) > 200 ? MOVE_PWM : SLOW_PWM;
    carriage(err > 0 ? pwm : -pwm);
    if (err > 0 && digitalRead(LIM_BOTTOM) == LOW) break;
    if (err < 0 && digitalRead(LIM_TOP) == LOW) break;
    if (millis() - t0 > 40000) fault("move timeout");
  }
  carriage(0);
}

void homeTop() {
  unsigned long t0 = millis();
  carriage(-MOVE_PWM);
  while (digitalRead(LIM_TOP) == HIGH) {
    if (estop()) fault("E-stop");
    if (millis() - t0 > 40000) fault("homing timeout");
  }
  carriage(0);
  noInterrupts(); encCount = 0; interrupts();
}

long measureRail() {                    // sweep down with blade retracted, count to bottom stop
  unsigned long t0 = millis();
  carriage(MOVE_PWM);
  while (digitalRead(LIM_BOTTOM) == HIGH) {
    if (estop()) fault("E-stop");
    if (millis() - t0 > 40000) fault("rail sweep timeout");
  }
  carriage(0);
  return encCount;
}

// Needle probe: returns bark thickness in mm, or -1 on failure
float gaugeBark() {
  const long chunk = lround(PROBE_STEP_MM * STEPS_PER_MM);
  float hist[4] = {0, 0, 0, 0};
  long surface = -1;
  probePosSteps = 0;
  while (probePosSteps < MAX_PROBE_MM * STEPS_PER_MM) {
    if (estop()) fault("E-stop");
    stepMotor(PROBE_STEP, PROBE_DIR, true, chunk, 150);
    probePosSteps += chunk;
    float f = scale.get_units(1);
    for (int i = 0; i < 3; i++) hist[i] = hist[i + 1];
    hist[3] = f;
    if (surface < 0 && f > CONTACT_G) surface = probePosSteps;
    if (surface >= 0 && probePosSteps - surface > 3 * chunk && (hist[3] - hist[0]) > WOOD_RISE_G) {
      long woodPos = probePosSteps - chunk;          // rise began about one step earlier
      retractProbe();
      return (woodPos - surface) / STEPS_PER_MM;
    }
  }
  retractProbe();
  return -1;
}

// ---------------- Tree tag (NTAG213) ----------------
// page 4: 'S','T', version, 0
// page 5: T1, T2, T3 in 0.1 mm units, spare
// page 6: day number (uint16, days since 2020-01-01), railCounts/10 (uint16)
uint16_t today() { return (uint16_t)(rtc.now().unixtime() / 86400UL - 18262UL); }

bool readTag(bool &valid) {
  valid = false;
  tagPresent = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, tagUid, &tagUidLen, 1500);
  if (!tagPresent) return false;
  uint8_t p4[32], p5[32], p6[32];
  if (!nfc.ntag2xx_ReadPage(4, p4) || !nfc.ntag2xx_ReadPage(5, p5) || !nfc.ntag2xx_ReadPage(6, p6)) return true;
  if (p4[0] != 'S' || p4[1] != 'T' || p4[2] != 2) return true;
  uint16_t day = p6[0] | (p6[1] << 8);
  long rc = (long)(p6[2] | (p6[3] << 8)) * 10;
  int age = (int)today() - (int)day;
  if (age < 0 || age > RECORD_MAX_AGE_DAYS || rc <= 0) return true;
  for (int i = 0; i < 3; i++) barkT[i] = p5[i] / 10.0;
  for (int i = 0; i < 3; i++) if (barkT[i] < MIN_VALID_T || barkT[i] > MAX_VALID_T) return true;
  railCounts = rc;
  valid = true;
  return true;
}

void writeTag() {
  if (!tagPresent) return;
  uint8_t p4[4] = {'S', 'T', 2, 0};
  uint8_t p5[4] = {(uint8_t)lround(barkT[0] * 10), (uint8_t)lround(barkT[1] * 10), (uint8_t)lround(barkT[2] * 10), 0};
  uint16_t d = today(); uint16_t rc = railCounts / 10;
  uint8_t p6[4] = {(uint8_t)(d & 0xFF), (uint8_t)(d >> 8), (uint8_t)(rc & 0xFF), (uint8_t)(rc >> 8)};
  if (!(nfc.ntag2xx_WritePage(4, p4) && nfc.ntag2xx_WritePage(5, p5) && nfc.ntag2xx_WritePage(6, p6)))
    logMsg("WARN: tag write failed - will re-gauge next time");
}

// ---------------- Depth profile ----------------
float barkAt(float frac) {                  // piecewise-linear through the 3 gauge points
  if (frac <= GAUGE_FRAC[0]) return barkT[0];
  if (frac >= GAUGE_FRAC[2]) return barkT[2];
  int a = frac < GAUGE_FRAC[1] ? 0 : 1;
  float u = (frac - GAUGE_FRAC[a]) / (GAUGE_FRAC[a + 1] - GAUGE_FRAC[a]);
  return barkT[a] + (barkT[a + 1] - barkT[a]) * u;
}

// ---------------- Main tapping cycle ----------------
void tapTree() {
  digitalWrite(LED, HIGH);
  setBladeDepth(0);
  retractProbe();

  bool valid = false;
  if (!readTag(valid)) logMsg("WARN: no tree tag - gauging, result not stored");

  homeTop();
  if (!valid) {
    railCounts = measureRail();
    if (railCounts < 500) fault("rail too short / end stops wrong");
    for (int i = 2; i >= 0; i--) {                        // gauge on the way back up
      long probeAt = lround(GAUGE_FRAC[i] * railCounts);
      moveTo(max(0L, probeAt - PROBE_OFFSET_COUNTS));
      float t = gaugeBark();
      if (t < MIN_VALID_T || t > MAX_VALID_T) fault("bark reading out of range - check probe");
      barkT[i] = t;
    }
    writeTag();
    homeTop();
  }
  logMsg("T = " + String(barkT[0], 1) + ", " + String(barkT[1], 1) + ", " + String(barkT[2], 1) + " mm");

  // Plunge at the high end, then cut downhill
  setBladeDepth(barkAt(0) - RESIDUAL_BARK_MM);
  unsigned long t0 = millis(); float peakA = 0;
  carriage(CUT_PWM);
  while (digitalRead(LIM_BOTTOM) == HIGH) {
    if (estop()) fault("E-stop");
    float a = motorAmps(); peakA = max(peakA, a);
    if (a > CURRENT_LIMIT_A) fault("motor overload - blade jammed?");
    float frac = constrain((float)encCount / railCounts, 0.0f, 1.0f);
    setBladeDepth(barkAt(frac) - RESIDUAL_BARK_MM);
    if (millis() - t0 > 60000) fault("cut timeout");
  }
  carriage(0);
  setBladeDepth(0);
  homeTop();

  logMsg("Done in " + String((millis() - t0) / 1000.0, 1) + " s, peak " + String(peakA, 1) + " A");
  digitalWrite(LED, LOW);
  beep(1);
}

void setup() {
  Serial.begin(115200);
  bt.begin("SmartTapper");
  int outs[] = {CAR_RPWM, CAR_LPWM, DEPTH_STEP, DEPTH_DIR, PROBE_STEP, PROBE_DIR, STEPPER_EN, LED, BUZZER};
  for (int p : outs) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  int ins[] = {LIM_TOP, LIM_BOTTOM, BTN_START, ENC_A, ENC_B};
  for (int p : ins) pinMode(p, INPUT_PULLUP);
  pinMode(BTN_ESTOP, INPUT);                       // external pull-up
  attachInterrupt(digitalPinToInterrupt(ENC_A), onEncoder, RISING);

  Wire.begin(21, 22);
  if (!rtc.begin()) logMsg("WARN: RTC missing - tag records will not expire correctly");
  nfc.begin();
  if (!nfc.getFirmwareVersion()) logMsg("WARN: NFC reader not found");
  else nfc.SAMConfig();

  scale.begin(HX_DOUT, HX_SCK);
  scale.set_scale(105.0);     // CALIBRATE with a known weight so that units = grams
  scale.tare();
  beep(2);
  logMsg("Smart Tapper ready");
}

void loop() {
  if (digitalRead(BTN_START) == LOW && !estop()) {
    delay(50);
    while (digitalRead(BTN_START) == LOW) delay(10);   // wait for release (fixes v1 bug)
    tapTree();
  }
}
