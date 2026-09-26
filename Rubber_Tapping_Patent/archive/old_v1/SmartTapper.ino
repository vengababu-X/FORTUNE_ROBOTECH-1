/*
 * Smart Clamp-On Spiral Tapper (SCST) - prototype controller
 * Board: Arduino Nano   Library needed: "HX711" by Bogdan Necula (Library Manager)
 *
 * All THRESHOLDS below are starting values. They MUST be calibrated on real
 * rubber bark (see Section 9 of the project document).
 */
#include <HX711.h>

// ---------------- Pins ----------------
const int CAR_RPWM = 5,  CAR_LPWM = 6;      // BTS7960 carriage motor (RPWM = down the rail)
const int DEPTH_STEP = 2, DEPTH_DIR = 3;    // A4988 - blade depth stepper
const int PROBE_STEP = 4, PROBE_DIR = 7;    // A4988 - needle probe stepper
const int STEPPER_EN = 8;                   // LOW = both A4988 enabled
const int LIM_TOP = 9, LIM_BOTTOM = 10;     // limit switches (to GND, INPUT_PULLUP)
const int BTN_START = 11;                   // start / emergency stop button
const int BUZZER = 12, LED = 13;
const int CURRENT_PIN = A0;                 // ACS712-20A output
const int HX_DOUT = A1, HX_SCK = A2;        // load cell amplifier

// ---------------- Mechanics ----------------
const float STEPS_PER_MM = 160.0;   // 2 mm lead screw, 20 steps/rev, 1/16 microstep
const float RESIDUAL_BARK_MM = 1.0; // bark left over cambium (c)
const float MAX_PROBE_MM = 15.0;    // safety limit on probe travel
const float MAX_DEPTH_MM = 10.0;    // safety limit on blade depth
const int   N_POINTS = 3;           // bark gauging points along the cut
const unsigned long RAIL_TIME_MS = 12000;  // time for carriage to travel full rail (measure!)

// ---------------- Sensor thresholds (CALIBRATE) ----------------
const float CONTACT_G = 30.0;       // force when needle touches bark surface
const float WOOD_RISE_G = 400.0;    // extra force jump in 0.2 mm that means "wood reached"
const int   CURRENT_LIMIT_RAW = 700;// ADC value for motor overload (~8 A on ACS712-20A)

HX711 scale;
float barkT[N_POINTS];              // measured bark thickness at each point (mm)
long  depthPosSteps = 0;            // current blade position (0 = blade flush with skid)

// ---------------- Helpers ----------------
void beep(int n) { for (int i = 0; i < n; i++) { digitalWrite(BUZZER, HIGH); delay(120); digitalWrite(BUZZER, LOW); delay(120); } }

bool stopPressed() { return digitalRead(BTN_START) == LOW; }

void stepMotor(int stepPin, int dirPin, bool forward, long steps, int usDelay = 400) {
  digitalWrite(dirPin, forward ? HIGH : LOW);
  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH); delayMicroseconds(usDelay);
    digitalWrite(stepPin, LOW);  delayMicroseconds(usDelay);
  }
}

void setBladeDepth(float mm) {
  mm = constrain(mm, 0, MAX_DEPTH_MM);
  long target = (long)(mm * STEPS_PER_MM);
  long diff = target - depthPosSteps;
  stepMotor(DEPTH_STEP, DEPTH_DIR, diff > 0, labs(diff), 300);
  depthPosSteps = target;
}

void carriage(int speed) {          // +speed = down the rail, -speed = up, 0 = stop
  if (speed > 0)      { analogWrite(CAR_RPWM, speed); analogWrite(CAR_LPWM, 0); }
  else if (speed < 0) { analogWrite(CAR_RPWM, 0); analogWrite(CAR_LPWM, -speed); }
  else                { analogWrite(CAR_RPWM, 0); analogWrite(CAR_LPWM, 0); }
}

void fault(const char *msg) {
  carriage(0);
  setBladeDepth(0);
  Serial.print("FAULT: "); Serial.println(msg);
  while (true) { beep(3); delay(1000); }   // stay stopped until reset
}

// Push needle into bark, return bark thickness in mm (or -1 on failure)
float gaugeBark() {
  long pos = 0, surface = -1;
  float prevF = scale.get_units(1);
  const long stepChunk = (long)(0.2 * STEPS_PER_MM);  // move 0.2 mm, then read force
  while (pos < MAX_PROBE_MM * STEPS_PER_MM) {
    stepMotor(PROBE_STEP, PROBE_DIR, true, stepChunk, 600);
    pos += stepChunk;
    float f = scale.get_units(2);
    if (surface < 0 && f > CONTACT_G) surface = pos;          // touched bark
    if (surface >= 0 && (f - prevF) > WOOD_RISE_G) {           // sudden rise = wood
      stepMotor(PROBE_STEP, PROBE_DIR, false, pos, 300);       // retract fully
      return (pos - surface) / STEPS_PER_MM;
    }
    prevF = f;
  }
  stepMotor(PROBE_STEP, PROBE_DIR, false, pos, 300);
  return -1;
}

void homeCarriage() {
  carriage(-180);
  unsigned long t0 = millis();
  while (digitalRead(LIM_TOP) == HIGH) {
    if (millis() - t0 > RAIL_TIME_MS * 2) fault("homing timeout");
  }
  carriage(0);
}

// ---------------- Main tapping cycle ----------------
void tapTree() {
  digitalWrite(LED, HIGH);
  setBladeDepth(0);
  homeCarriage();

  // 1. Gauge bark at N points (probe sits in the strip that will be shaved off)
  unsigned long segment = RAIL_TIME_MS / (N_POINTS - 1);
  for (int i = 0; i < N_POINTS; i++) {
    float t = gaugeBark();
    if (t < 2.0 || t > 14.0) fault("bark reading out of range");
    barkT[i] = t;
    Serial.print("T"); Serial.print(i); Serial.print(" = "); Serial.println(t);
    if (i < N_POINTS - 1) { carriage(180); delay(segment); carriage(0); }
  }
  homeCarriage();

  // 2. Cut: blade depth = T - c, linearly interpolated between gauge points
  unsigned long t0 = millis();
  carriage(200);
  while (digitalRead(LIM_BOTTOM) == HIGH) {
    if (stopPressed()) fault("stopped by user");
    if (analogRead(CURRENT_PIN) > CURRENT_LIMIT_RAW) fault("motor overload");
    float frac = constrain((float)(millis() - t0) / RAIL_TIME_MS, 0, 1);
    float idx = frac * (N_POINTS - 1);
    int a = (int)idx; int b = min(a + 1, N_POINTS - 1);
    float T = barkT[a] + (barkT[b] - barkT[a]) * (idx - a);
    setBladeDepth(T - RESIDUAL_BARK_MM);
    if (millis() - t0 > RAIL_TIME_MS * 2) fault("cut timeout");
  }
  carriage(0);

  // 3. Retract and return
  setBladeDepth(0);
  homeCarriage();
  digitalWrite(LED, LOW);
  beep(1);
}

void setup() {
  Serial.begin(9600);
  int outs[] = {CAR_RPWM, CAR_LPWM, DEPTH_STEP, DEPTH_DIR, PROBE_STEP, PROBE_DIR, STEPPER_EN, BUZZER, LED};
  for (int p : outs) pinMode(p, OUTPUT);
  pinMode(LIM_TOP, INPUT_PULLUP); pinMode(LIM_BOTTOM, INPUT_PULLUP); pinMode(BTN_START, INPUT_PULLUP);
  digitalWrite(STEPPER_EN, LOW);
  scale.begin(HX_DOUT, HX_SCK);
  scale.set_scale(420.0);   // CALIBRATE with a known weight so units = grams
  scale.tare();
  beep(2);
}

void loop() {
  if (stopPressed()) {
    delay(300);            // debounce, then run one full tapping cycle
    tapTree();
  }
}
