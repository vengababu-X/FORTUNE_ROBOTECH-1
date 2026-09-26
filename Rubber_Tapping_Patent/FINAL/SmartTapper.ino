/*
 * SMART TAPPER - final prototype controller
 * Board : ESP32 DevKit (ESP32-WROOM-32, classic ESP32 - needed for BluetoothSerial)
 * Core  : Arduino-ESP32 core 3.x
 * Libraries (Arduino Library Manager): "HX711" by Bogdan Necula, "Adafruit PN532", "RTClib"
 *
 * Settings are stored in flash and can be changed from a phone over Bluetooth
 * (any "Bluetooth serial terminal" app, device name "SmartTapper"). Send HELP.
 * All settings start from safe guesses and MUST be calibrated (project file, Sections 10-12).
 */
#include <Wire.h>
#include <HX711.h>
#include <Adafruit_PN532.h>
#include <RTClib.h>
#include <BluetoothSerial.h>
#include <Preferences.h>

// ---------------- Pins ----------------
const int CAR_RPWM = 25, CAR_LPWM = 26;       // BTS7960: RPWM = carriage DOWN the rail (cutting)
const int ENC_A = 32, ENC_B = 33;             // carriage motor hall encoder
const int DEPTH_STEP = 14, DEPTH_DIR = 27;    // TMC2209 #1 - blade depth (DIR HIGH = toward bark)
const int PROBE_STEP = 13, PROBE_DIR = 4;     // TMC2209 #2 - needle probe (DIR HIGH = toward bark)
const int STEPPER_EN = 23;                    // LOW = drivers enabled
const int DEPTH_HOME = 16, PROBE_HOME = 17;   // micro-switches closed (to GND) at fully retracted position
const int LIM_TOP = 18, LIM_BOTTOM = 19;      // adjustable end stops on rail (to GND)
const int BTN_ESTOP = 35;                     // E-stop aux contact to GND; input-only pin: external 10k pull-up to 3.3 V
const int WORK_LAMP = 2;                      // MOSFET for LED work lamp
const int CURRENT_PIN = 36;                   // ACS712-20A via 10k/20k divider
const int HX_DOUT = 39, HX_SCK = 5;           // HX711 with RATE pin HIGH (80 samples/s)
const int NFC_IRQ = 34, NFC_RESET = 15;       // PN532 (I2C mode)
// I2C bus (SDA 21, SCL 22): PN532, DS3231 RTC, PCF8574 I/O expander (0x20), INA219 battery monitor (0x40)
const uint8_t PCF_ADDR = 0x20, INA_ADDR = 0x40;
const uint8_t PCF_START = 0, PCF_LED = 1, PCF_BUZZER = 2;   // PCF8574 bits (START button to GND)

// ---------------- Fixed mechanics ----------------
const float STEPS_PER_MM = 800.0;       // 200 step motor, 1/8 microstep (TMC2209 standalone default), Tr8x2 screw
const float MAX_DEPTH_MM = 12.0;        // blade depth below skid, hard limit
const float MAX_PROBE_MM = 40.0;        // probe travel from home, hard limit
const float MIN_VALID_T = 3.0, MAX_VALID_T = 15.0;
const float GAUGE_FRAC[3] = {0.10, 0.50, 0.90};
const int   CUT_PWM = 200, MOVE_PWM = 170, SLOW_PWM = 110;
const float BATT_MIN_START_V = 10.8, BATT_MIN_RUN_V = 10.0;   // 3S Li-ion
const int   RECORD_MAX_AGE_DAYS = 30;
const uint8_t TAG_VERSION = 3;

// ---------------- Settings (flash, changeable over Bluetooth) ----------------
Preferences prefs;
struct Settings {
  float c;          // residual bark over cambium, mm (0.5 .. 1.5)
  float flushMm;    // blade travel from home switch to "tip flush with skid", mm
  float contactG;   // probe force at bark contact, g
  float woodRiseG;  // probe force rise over 0.3 mm meaning wood, g
  float currentA;   // carriage overload limit, A
  long  probeOffset;// encoder counts the probe is ahead of the blade
  float hxScale;    // HX711 counts per gram
} S;

void loadSettings() {
  prefs.begin("tapper", false);
  S.c = prefs.getFloat("c", 1.0);
  S.flushMm = prefs.getFloat("flush", 5.0);
  S.contactG = prefs.getFloat("contact", 50);
  S.woodRiseG = prefs.getFloat("wood", 600);
  S.currentA = prefs.getFloat("amps", 10);
  S.probeOffset = prefs.getLong("poff", 300);
  S.hxScale = prefs.getFloat("hx", 105);
}
void saveSettings() {
  prefs.putFloat("c", S.c); prefs.putFloat("flush", S.flushMm); prefs.putFloat("contact", S.contactG);
  prefs.putFloat("wood", S.woodRiseG); prefs.putFloat("amps", S.currentA); prefs.putLong("poff", S.probeOffset);
  prefs.putFloat("hx", S.hxScale);
}

// ---------------- Objects / state ----------------
HX711 scale;
Adafruit_PN532 nfc(NFC_IRQ, NFC_RESET);
RTC_DS3231 rtc;
BluetoothSerial bt;
bool rtcOk = false, nfcOk = false;

volatile long encCount = 0;
long railCounts = 0;
long depthPos = 0, probePos = 0;        // steps from home switches
float barkT[3];
uint16_t recordDay = 0;                 // day the bark was gauged (kept, not refreshed by tapping)
uint8_t tagUid[7]; uint8_t tagUidLen = 0; bool tagPresent = false;
uint8_t pcfOut = 0xFF;                  // PCF8574: 1 = input / off

void IRAM_ATTR onEncoder() { encCount += (digitalRead(ENC_B) ? -1 : 1); }

// ---------------- I/O helpers ----------------
void logMsg(const String &s) { Serial.println(s); bt.println(s); }
void pcfWrite() { Wire.beginTransmission(PCF_ADDR); Wire.write(pcfOut); Wire.endTransmission(); }
void pcfSet(uint8_t bit, bool on) {          // LED / buzzer are active LOW (sink current)
  if (on) pcfOut &= ~(1 << bit); else pcfOut |= (1 << bit);
  pcfWrite();
}
bool startPressed() {
  Wire.requestFrom(PCF_ADDR, (uint8_t)1);
  return Wire.available() && !(Wire.read() & (1 << PCF_START));
}
void beep(int n, int ms = 120) { for (int i = 0; i < n; i++) { pcfSet(PCF_BUZZER, true); delay(ms); pcfSet(PCF_BUZZER, false); delay(ms); } }
bool estop() { return digitalRead(BTN_ESTOP) == LOW; }

float batteryV() {                            // INA219 bus voltage register
  Wire.beginTransmission(INA_ADDR); Wire.write(0x02); Wire.endTransmission();
  Wire.requestFrom(INA_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  uint16_t raw = (Wire.read() << 8) | Wire.read();
  return (raw >> 3) * 0.004;
}
float motorAmps() {
  float mv = analogReadMilliVolts(CURRENT_PIN) / 0.667;
  return fabs((mv - 2500.0) / 100.0);
}
void carriage(int speed) {
  if (speed > 0)      { analogWrite(CAR_RPWM, speed); analogWrite(CAR_LPWM, 0); }
  else if (speed < 0) { analogWrite(CAR_RPWM, 0);     analogWrite(CAR_LPWM, -speed); }
  else                { analogWrite(CAR_RPWM, 0);     analogWrite(CAR_LPWM, 0); }
}
void pulses(int stepPin, int dirPin, bool fwd, long n, int usHalf = 60) {
  digitalWrite(dirPin, fwd ? HIGH : LOW);
  for (long i = 0; i < n; i++) { digitalWrite(stepPin, HIGH); delayMicroseconds(usHalf); digitalWrite(stepPin, LOW); delayMicroseconds(usHalf); }
}

// Home an axis: retract until switch closes, back off 0.5 mm, re-touch slowly
bool homeAxis(int stepPin, int dirPin, int homePin, long &pos) {
  long guard = lround(60 * STEPS_PER_MM);
  while (digitalRead(homePin) == HIGH && guard-- > 0) pulses(stepPin, dirPin, false, 1, 80);
  if (guard <= 0) return false;
  pulses(stepPin, dirPin, true, lround(0.5 * STEPS_PER_MM), 150);
  guard = lround(1.0 * STEPS_PER_MM);
  while (digitalRead(homePin) == HIGH && guard-- > 0) pulses(stepPin, dirPin, false, 1, 400);
  pos = 0;
  return guard > 0;
}

void setBladeDepth(float mm) {                // mm below skid; negative = retracted behind skid
  mm = constrain(mm, -S.flushMm, MAX_DEPTH_MM);
  long target = lround((S.flushMm + mm) * STEPS_PER_MM);
  long diff = target - depthPos;
  if (diff) pulses(DEPTH_STEP, DEPTH_DIR, diff > 0, labs(diff));
  depthPos = target;
}
void retractBlade() { setBladeDepth(-S.flushMm); }
void retractProbe() { if (probePos > 0) pulses(PROBE_STEP, PROBE_DIR, false, probePos); probePos = 0; }

void fault(const char *msg) {
  carriage(0);
  if (!estop()) { retractProbe(); retractBlade(); }   // with E-stop pressed the drivers have no power
  logMsg(String("FAULT: ") + msg + " - release E-stop and switch off/on (axes re-home at start-up)");
  pcfSet(PCF_LED, false);
  while (true) { beep(3); delay(1500); }
}
void checkRun() {
  if (estop()) fault("E-stop pressed");
  float v = batteryV();
  if (v > 1 && v < BATT_MIN_RUN_V) fault("battery low");
}

// ---------------- Carriage ----------------
void moveTo(long target) {
  unsigned long t0 = millis();
  while (labs(target - encCount) > 5) {
    checkRun();
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
  while (digitalRead(LIM_TOP) == HIGH) { checkRun(); if (millis() - t0 > 40000) fault("homing timeout"); }
  carriage(0);
  noInterrupts(); encCount = 0; interrupts();
}
long sweepToBottom() {
  unsigned long t0 = millis();
  carriage(MOVE_PWM);
  while (digitalRead(LIM_BOTTOM) == HIGH) { checkRun(); if (millis() - t0 > 40000) fault("rail sweep timeout"); }
  carriage(0);
  return encCount;
}

// ---------------- Probe ----------------
float gaugeBark() {
  const long chunk = lround(0.1 * STEPS_PER_MM);
  float hist[4] = {0, 0, 0, 0};
  long surface = -1;
  while (probePos < MAX_PROBE_MM * STEPS_PER_MM) {
    checkRun();
    pulses(PROBE_STEP, PROBE_DIR, true, chunk, 150);
    probePos += chunk;
    float f = scale.get_units(1);
    for (int i = 0; i < 3; i++) hist[i] = hist[i + 1];
    hist[3] = f;
    if (surface < 0 && f > S.contactG) surface = probePos;
    if (surface >= 0 && probePos - surface > 3 * chunk && (hist[3] - hist[0]) > S.woodRiseG) {
      long wood = probePos - chunk;
      retractProbe();
      return (wood - surface) / STEPS_PER_MM;
    }
  }
  retractProbe();
  return -1;
}

// ---------------- Tree tag (NTAG213 pages 4-6) ----------------
uint16_t today() { return rtcOk ? (uint16_t)(rtc.now().unixtime() / 86400UL - 18262UL) : 0; }

bool waitForTag() {                          // operator holds the control box against the tag
  unsigned long t0 = millis();
  while (millis() - t0 < 6000) {
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, tagUid, &tagUidLen, 500)) { beep(1, 60); return true; }
    beep(1, 30);
  }
  return false;
}
bool readRecord() {                          // true = valid, fresh record loaded
  uint8_t p4[32], p5[32], p6[32];
  if (!nfc.ntag2xx_ReadPage(4, p4) || !nfc.ntag2xx_ReadPage(5, p5) || !nfc.ntag2xx_ReadPage(6, p6)) return false;
  if (p4[0] != 'S' || p4[1] != 'T' || p4[2] != TAG_VERSION) return false;
  recordDay = p6[0] | (p6[1] << 8);
  int age = (int)today() - (int)recordDay;
  long rc = (long)(p6[2] | (p6[3] << 8)) * 10;
  if (!rtcOk || age < 0 || age > RECORD_MAX_AGE_DAYS || rc <= 0) return false;
  for (int i = 0; i < 3; i++) { barkT[i] = p5[i] / 10.0; if (barkT[i] < MIN_VALID_T || barkT[i] > MAX_VALID_T) return false; }
  railCounts = rc;
  return true;
}
void writeRecord(bool valid) {
  if (!tagPresent) return;
  uint8_t p4[4] = {'S', 'T', (uint8_t)(valid ? TAG_VERSION : 0), 0};
  uint8_t p5[4] = {(uint8_t)lround(barkT[0] * 10), (uint8_t)lround(barkT[1] * 10), (uint8_t)lround(barkT[2] * 10), 0};
  uint16_t d = recordDay, rc = railCounts / 10;
  uint8_t p6[4] = {(uint8_t)(d & 0xFF), (uint8_t)(d >> 8), (uint8_t)(rc & 0xFF), (uint8_t)(rc >> 8)};
  if (!(nfc.ntag2xx_WritePage(4, p4) && nfc.ntag2xx_WritePage(5, p5) && nfc.ntag2xx_WritePage(6, p6)))
    logMsg("WARN: tag write failed - will re-gauge next time");
}
String uidStr() { String s; for (int i = 0; i < tagUidLen; i++) { if (tagUid[i] < 16) s += "0"; s += String(tagUid[i], HEX); } return s; }

float barkAt(float frac) {
  if (frac <= GAUGE_FRAC[0]) return barkT[0];
  if (frac >= GAUGE_FRAC[2]) return barkT[2];
  int a = frac < GAUGE_FRAC[1] ? 0 : 1;
  float u = (frac - GAUGE_FRAC[a]) / (GAUGE_FRAC[a + 1] - GAUGE_FRAC[a]);
  return barkT[a] + (barkT[a + 1] - barkT[a]) * u;
}

// ---------------- Main tapping cycle ----------------
void tapTree() {
  float v = batteryV();
  if (v > 1 && v < BATT_MIN_START_V) { logMsg("Battery low - charge before tapping"); beep(4); return; }
  pcfSet(PCF_LED, true);
  digitalWrite(WORK_LAMP, HIGH);
  retractBlade(); retractProbe();

  tagPresent = nfcOk && waitForTag();
  bool valid = tagPresent && readRecord();
  if (!tagPresent) logMsg("WARN: no tree tag - bark will be gauged and not stored");

  homeTop();
  if (!valid) {
    railCounts = sweepToBottom();
    if (railCounts < 500) fault("rail too short or end stops set wrongly");
    for (int i = 2; i >= 0; i--) {
      long carriageAt = lround(GAUGE_FRAC[i] * railCounts) - S.probeOffset;
      if (carriageAt < 0) fault("probe offset larger than first gauge point - check setting poff");
      moveTo(carriageAt);
      float t = gaugeBark();
      if (t < MIN_VALID_T || t > MAX_VALID_T) fault("bark reading out of range - check needle and settings");
      barkT[i] = t;
    }
    recordDay = today();
    homeTop();
  }
  logMsg("Tree " + uidStr() + "  T = " + String(barkT[0], 1) + " / " + String(barkT[1], 1) + " / " + String(barkT[2], 1) + " mm  c = " + String(S.c, 1));

  setBladeDepth(barkAt(0) - S.c);                       // plunge at the high end
  unsigned long t0 = millis(); float peakA = 0;
  carriage(CUT_PWM);
  while (digitalRead(LIM_BOTTOM) == HIGH) {
    checkRun();
    float a = motorAmps(); peakA = max(peakA, a);
    if (a > S.currentA) fault("motor overload - blade jammed or blunt?");
    float frac = constrain((float)encCount / railCounts, 0.0f, 1.0f);
    setBladeDepth(barkAt(frac) - S.c);
    if (millis() - t0 > 60000) fault("cut timeout");
  }
  carriage(0);
  long measured = encCount;
  retractBlade();
  homeTop();

  // rail moved or end stops changed by more than 10 %? then force re-gauge next time
  bool stillValid = labs(measured - railCounts) <= railCounts / 10;
  writeRecord(stillValid);
  if (!stillValid) logMsg("NOTE: rail length changed - bark will be re-gauged next time");

  logMsg("Done " + String((millis() - t0) / 1000.0, 1) + " s, peak " + String(peakA, 1) + " A, battery " + String(batteryV(), 1) + " V");
  pcfSet(PCF_LED, false);
  digitalWrite(WORK_LAMP, LOW);
  beep(1);
}

// ---------------- Blade zero calibration (after every sharpening) ----------------
// Hold a flat steel plate against the skid roller, send ZERO, press START when the blade tip touches the plate.
void calibrateZero() {
  homeAxis(DEPTH_STEP, DEPTH_DIR, DEPTH_HOME, depthPos);
  logMsg("Advancing blade slowly - press START when tip touches the plate");
  long guard = lround(20 * STEPS_PER_MM);
  while (!startPressed() && guard > 0) { pulses(DEPTH_STEP, DEPTH_DIR, true, 16, 200); depthPos += 16; guard -= 16; delay(25); }
  S.flushMm = depthPos / STEPS_PER_MM;
  saveSettings();
  logMsg("Saved flush = " + String(S.flushMm, 2) + " mm");
  while (startPressed()) delay(10);
  retractBlade();
}

// ---------------- Bluetooth / serial commands ----------------
void handleCommand(String cmd) {
  cmd.trim(); cmd.toUpperCase();
  if (cmd == "HELP") logMsg("SHOW | SET C|CONTACT|WOOD|AMPS|POFF <v> | TARE | CALW <grams> | ZERO | FORCE (live probe force) | BATT");
  else if (cmd == "SHOW") logMsg("c=" + String(S.c, 2) + " flush=" + String(S.flushMm, 2) + " contact=" + String(S.contactG) + " wood=" + String(S.woodRiseG) +
                            " amps=" + String(S.currentA) + " poff=" + String(S.probeOffset) + " hx=" + String(S.hxScale, 2));
  else if (cmd.startsWith("SET ")) {
    int sp = cmd.indexOf(' ', 4); String key = cmd.substring(4, sp); float val = cmd.substring(sp + 1).toFloat();
    if (key == "C" && val >= 0.5 && val <= 1.5) S.c = val;
    else if (key == "CONTACT" && val > 0) S.contactG = val;
    else if (key == "WOOD" && val > 0) S.woodRiseG = val;
    else if (key == "AMPS" && val > 1 && val < 20) S.currentA = val;
    else if (key == "POFF" && val >= 0) S.probeOffset = (long)val;
    else { logMsg("Rejected: out of range"); return; }
    saveSettings(); logMsg("Saved");
  }
  else if (cmd == "TARE") { scale.tare(); logMsg("Tared"); }
  else if (cmd.startsWith("CALW ")) {                // put a known weight on the probe load cell first
    float g = cmd.substring(5).toFloat();
    if (g > 0) { scale.set_scale(1); float raw = scale.get_units(10); S.hxScale = raw / g; scale.set_scale(S.hxScale); saveSettings(); logMsg("Scale = " + String(S.hxScale, 2)); }
  }
  else if (cmd == "ZERO") calibrateZero();
  else if (cmd == "FORCE") { for (int i = 0; i < 20; i++) { logMsg(String(scale.get_units(1), 0) + " g"); delay(200); } }
  else if (cmd == "BATT") logMsg(String(batteryV(), 2) + " V");
  else logMsg("Unknown - send HELP");
}

void setup() {
  Serial.begin(115200);
  bt.begin("SmartTapper");
  int outs[] = {CAR_RPWM, CAR_LPWM, DEPTH_STEP, DEPTH_DIR, PROBE_STEP, PROBE_DIR, STEPPER_EN, WORK_LAMP};
  for (int p : outs) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  int ins[] = {LIM_TOP, LIM_BOTTOM, DEPTH_HOME, PROBE_HOME, ENC_A, ENC_B};
  for (int p : ins) pinMode(p, INPUT_PULLUP);
  pinMode(BTN_ESTOP, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), onEncoder, RISING);

  Wire.begin(21, 22);
  pcfWrite();
  loadSettings();
  rtcOk = rtc.begin();
  if (rtcOk && rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  if (!rtcOk) logMsg("WARN: RTC missing - bark will be gauged on every tree");
  nfc.begin();
  nfcOk = nfc.getFirmwareVersion() != 0;
  if (nfcOk) nfc.SAMConfig(); else logMsg("WARN: NFC reader not found");

  scale.begin(HX_DOUT, HX_SCK);
  scale.set_scale(S.hxScale);
  scale.tare();

  if (estop()) logMsg("Release E-stop to home the axes");
  while (estop()) delay(50);
  delay(300);
  if (!homeAxis(PROBE_STEP, PROBE_DIR, PROBE_HOME, probePos)) fault("probe home switch not found");
  if (!homeAxis(DEPTH_STEP, DEPTH_DIR, DEPTH_HOME, depthPos)) fault("blade home switch not found");
  beep(2);
  logMsg("Smart Tapper ready. Battery " + String(batteryV(), 1) + " V. Send HELP for commands.");
}

void loop() {
  if (bt.available()) handleCommand(bt.readStringUntil('\n'));
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));
  if (startPressed() && !estop()) {
    delay(50);
    while (startPressed()) delay(10);                 // act on release
    tapTree();
  }
}
