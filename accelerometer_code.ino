// CCCARE - Wearable accelerometer curl tracker with full-rep checking
// Sensor: Adafruit ADXL335
//   Vin -> 3.3V, GND -> GND, Xout -> A0, Yout -> A1, Zout -> A2  (3Vo and Test: nothing)
// Tape the sensor flat on the forearm near the wrist.
//
// A FULL rep = start with arm straight -> curl all the way up
//              -> straighten all the way back down.
// Flags:  "didn't reach the top"   = curled partway, then went back down
//         "didn't fully extend"    = curled again before straightening fully
//
// Serial Monitor at 115200 baud:
//   c = calibrate (arm DOWN, then arm UP)
//   s = start / stop a set

#include <math.h>

const int PIN_X = A0;
const int PIN_Y = A1;
const int PIN_Z = A2;

// ---------- Settings you can tune (all in % of your calibrated range) ----------
const int TARGET_REPS = 10;
const float TOP_PCT = 85.0;        // reached the top
const float START_PCT = 25.0;      // a curl has started
const float BOTTOM_PCT = 15.0;     // arm is back down (fully extended)
const float LEFT_TOP_PCT = 60.0;   // dropped below this = lowering from the top
const unsigned long MIN_REP_MS = 400;
const float FILTER_ALPHA = 0.2;    // smoothing (smaller = smoother, slower)

// ---------- Sensor constants ----------
const unsigned long SAMPLE_MS = 20;   // 50 Hz
const float VREF = 5.0;
const float ADC_MAX = 16383.0;        // 14-bit
const float ZERO_G_V = 1.65;
const float V_PER_G = 0.330;

struct Vec { float x, y, z; };
Vec g = {0, 0, 1};
Vec calDown, calUp;
float calSpanDeg = 0;
bool calibrated = false;

enum Phase { WAIT_DOWN, READY, CURLING_UP, AT_TOP, LOWERING };
Phase phase = WAIT_DOWN;

bool inSet = false;
int fullReps = 0;
int missedTop = 0;
int notExtended = 0;
unsigned long totalRepMs = 0;
unsigned long repStart = 0;
unsigned long setStart = 0;
unsigned long lastSample = 0;

float toG(int raw) {
  return (raw * VREF / ADC_MAX - ZERO_G_V) / V_PER_G;
}

Vec readRaw() {
  Vec v;
  v.x = toG(analogRead(PIN_X));
  v.y = toG(analogRead(PIN_Y));
  v.z = toG(analogRead(PIN_Z));
  return v;
}

float angleBetween(Vec a, Vec b) {
  float dot = a.x * b.x + a.y * b.y + a.z * b.z;
  float la = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  float lb = sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
  if (la < 0.01 || lb < 0.01) return 0;
  float c = dot / (la * lb);
  if (c > 1) c = 1;
  if (c < -1) c = -1;
  return acos(c) * 180.0 / PI;
}

Vec measure(const char* instruction) {
  Serial.println(instruction);
  for (int s = 3; s > 0; s--) {
    Serial.print("  hold still... ");
    Serial.println(s);
    delay(1000);
  }
  Serial.println("  measuring...");
  Vec sum = {0, 0, 0};
  int n = 0;
  unsigned long start = millis();
  while (millis() - start < 1000) {
    Vec r = readRaw();
    sum.x += r.x; sum.y += r.y; sum.z += r.z;
    n++;
    delay(10);
  }
  Vec avg = {sum.x / n, sum.y / n, sum.z / n};
  return avg;
}

void calibrate() {
  Serial.println();
  Serial.println("=== CALIBRATION ===");
  calDown = measure("Arm DOWN, fully straight.");
  calUp = measure("Arm UP, fully curled.");

  calSpanDeg = angleBetween(calDown, calUp);
  if (calSpanDeg < 30) {
    calibrated = false;
    Serial.print("FAILED: arm only moved ");
    Serial.print(calSpanDeg, 0);
    Serial.println(" deg. Check the sensor is taped firmly and type c again.");
  } else {
    calibrated = true;
    g = calDown;
    Serial.print("Calibrated! Range of motion: ");
    Serial.print(calSpanDeg, 0);
    Serial.println(" deg. Type s to start a set.");
  }
  lastSample = millis();
}

void startSet() {
  fullReps = missedTop = notExtended = 0;
  totalRepMs = 0;
  phase = WAIT_DOWN;
  setStart = millis();
  inSet = true;
  Serial.println();
  Serial.println("=== SET STARTED === (start with your arm straight; type s to stop)");
}

void endSet() {
  inSet = false;
  Serial.println();
  Serial.println("=== SET SUMMARY ===");
  Serial.print("Full reps:              ");
  Serial.print(fullReps);
  Serial.print(" / ");
  Serial.println(TARGET_REPS);
  Serial.print("Didn't reach the top:   ");
  Serial.println(missedTop);
  Serial.print("Didn't fully extend:    ");
  Serial.println(notExtended);
  Serial.print("Avg full rep time:      ");
  if (fullReps > 0) {
    Serial.print(totalRepMs / (float)fullReps / 1000.0, 2);
    Serial.println(" s");
  } else {
    Serial.println("-");
  }
  Serial.print("Set duration:           ");
  Serial.print((millis() - setStart) / 1000.0, 1);
  Serial.println(" s");
  Serial.println("Type s for another set, c to recalibrate.");
}

void trackReps(unsigned long now) {
  // 0% = arm down, 100% = arm up
  float p = angleBetween(g, calDown) / calSpanDeg * 100.0;

  switch (phase) {
    case WAIT_DOWN:
      if (p < BOTTOM_PCT) phase = READY;
      break;

    case READY:                         // arm straight, waiting for a curl
      if (p > START_PCT) {
        phase = CURLING_UP;
        repStart = now;
      }
      break;

    case CURLING_UP:                    // on the way up
      if (p >= TOP_PCT) {
        phase = AT_TOP;
      } else if (p < BOTTOM_PCT) {
        missedTop++;
        Serial.println("  incomplete: didn't reach the top");
        phase = READY;
      }
      break;

    case AT_TOP:                        // reached the top
      if (p < LEFT_TOP_PCT) phase = LOWERING;
      break;

    case LOWERING:                      // on the way down
      if (p < BOTTOM_PCT) {
        unsigned long dur = now - repStart;
        phase = READY;
        if (dur < MIN_REP_MS) break;    // too quick, ignore as noise
        fullReps++;
        totalRepMs += dur;
        Serial.print("REP ");
        Serial.print(fullReps);
        Serial.print("   time: ");
        Serial.print(dur / 1000.0, 2);
        Serial.print(" s");
        if (fullReps == TARGET_REPS) Serial.print("   << target reached!");
        Serial.println();
      } else if (p >= TOP_PCT) {
        notExtended++;
        Serial.println("  incomplete: didn't fully extend before curling again");
        repStart = now;
        phase = AT_TOP;
      }
      break;
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) {}
  analogReadResolution(14);
  Serial.println("CCCARE wearable accelerometer tracker (full-rep check)");
  Serial.println("Type c to calibrate, then s to start a set.");
  lastSample = millis();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'c' || ch == 'C') {
      if (inSet) endSet();
      calibrate();
    } else if (ch == 's' || ch == 'S') {
      if (inSet) endSet();
      else if (calibrated) startSet();
      else Serial.println("Calibrate first: type c");
    }
  }

  unsigned long now = millis();
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    Vec r = readRaw();
    g.x += FILTER_ALPHA * (r.x - g.x);
    g.y += FILTER_ALPHA * (r.y - g.y);
    g.z += FILTER_ALPHA * (r.z - g.z);
    if (inSet) trackReps(now);
  }
}