// ============================================================
//  REC Group 1 - CONTROLLER (Granular PWM Control + Turn Kick)
//  Board: ESP32 DevKit V1 (30-pin)
//  Sends 16-bit PWM drive + gripper commands to the robot.
// ============================================================
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ############################################################
// #                                                          #
// #   SECTION 1 - MANUAL TUNING VARIABLES                    #
// #                                                          #
// ############################################################

// ---------- 1.1 RADIO ----------
const uint16_t TEAM_KEY     = 0x5A27;
const uint8_t  WIFI_CHANNEL = 11;

// ---------- 1.2 STICK DIRECTION ----------
const bool INVERT_FORWARD = false;
const bool INVERT_TURN    = false;
const bool LED_ON_WHEN_OPEN = true;
const bool START_IN_PRECISE_MODE = false;

// ---------- 1.3 GLOBAL LIMITS ----------
const float PWM_CAP = 235;
const float TURN_MAX_RATIO_OF_FWD = 0.5f;

const float STICK_SMOOTHING = 0.35f;
const float TURN_SMOOTHING  = 0.18f;

// ---------- 1.4 STICTION SPIKE (KICK-START TURNING) ----------
const uint32_t SPIKE_DURATION_MS = 150; // Duration (ms) of kick pulse when turning starts
const int      SPIKE_BOOST_PWM    = 40;  // Extra PWM added to turnMin during the spike

// ============================================================
//  COARSE MODE (Fast driving)
// ============================================================
const int   COARSE_FWD_MIN_PWM = 170;
const int   COARSE_FWD_MAX_PWM = 235;
const int   COARSE_TURN_MIN_PWM = 20;
const int   COARSE_TURN_MAX_PWM = 34;
const int   COARSE_STEER_MAX_PWM = 40;

const float COARSE_FWD_COEFF    = 1.00f;
const float COARSE_REVERSE_FACTOR = 0.85f;
const float COARSE_TURN_COEFF_PIVOT  = 0.70f;
const float COARSE_TURN_COEFF_MOVING = 0.50f;

const float COARSE_FWD_DEADZONE         = 0.15f;
const float COARSE_TURN_DEADZONE_PIVOT  = 0.60f;
const float COARSE_TURN_DEADZONE_MOVING = 0.60f;

const float COARSE_FWD_EXPO  = 0.15f;
const float COARSE_TURN_EXPO = 1.00f;

const float COARSE_ACCEL = 700;
const float COARSE_DECEL = 1200;

const float COARSE_LEFT_TRIM  = 1.00f;
const float COARSE_RIGHT_TRIM = 1.00f;

// ============================================================
//  PRECISE MODE (Slow, fine alignment)
// ============================================================
const int   PRECISE_FWD_MIN_PWM = 55;
const int   PRECISE_FWD_MAX_PWM = 65;
const int   PRECISE_TURN_MIN_PWM = 20;
const int   PRECISE_TURN_MAX_PWM = 22;
const int   PRECISE_STEER_MAX_PWM = 15;

const float PRECISE_FWD_COEFF    = 1.00f;
const float PRECISE_REVERSE_FACTOR = 1.00f;
const float PRECISE_TURN_COEFF_PIVOT  = 0.80f;
const float PRECISE_TURN_COEFF_MOVING = 0.40f;

const float PRECISE_FWD_DEADZONE         = 0.12f;
const float PRECISE_TURN_DEADZONE_PIVOT  = 0.55f;
const float PRECISE_TURN_DEADZONE_MOVING = 0.55f;

const float PRECISE_FWD_EXPO  = 0.80f;
const float PRECISE_TURN_EXPO = 1.00f;

const float PRECISE_ACCEL = 600;
const float PRECISE_DECEL = 1500;

const float PRECISE_LEFT_TRIM  = 1.00f;
const float PRECISE_RIGHT_TRIM = 1.00f;

// ---------- 1.5 STICK SAFETY ----------
const float    ARM_STICK_ZONE = 0.10f;
const uint32_t RECAL_HOLD_MS  = 2000;
const uint32_t ARM_MS         = 300;
const int CENTER_MIN   = 1400;
const int CENTER_MAX   = 2800;
const int STABLE_SPAN  = 150;

// ---------- 1.6 PINS ----------
const int PIN_JOY_FWD  = 34;  // module pin labelled VRX  -> forward / back
const int PIN_JOY_TURN = 35;  // module pin labelled VRY  -> left / right
const int PIN_JOY_BTN  = 23;
const int PIN_BTN_G1   = 19;
const int PIN_BTN_G2   = 18;
const int PIN_LED_G1   = 32;
const int PIN_LED_G2   = 33;

// ============================================================
//  INTERNAL CALCULATIONS & STRUCTS
// ============================================================

struct Button { int pin; bool prev; uint32_t last; };

struct ModeCfg {
  int   fwdMin, fwdMax, turnMin, turnMax, steerMax;
  float fwdCoeff, reverseFactor, turnCoeffPivot, turnCoeffMoving;
  float fwdDeadzone, turnDzPivot, turnDzMoving;
  float fwdExpo, turnExpo;
  float accel, decel;
  float leftTrim, rightTrim;
};

const ModeCfg COARSE = {
  COARSE_FWD_MIN_PWM, COARSE_FWD_MAX_PWM, COARSE_TURN_MIN_PWM, COARSE_TURN_MAX_PWM, COARSE_STEER_MAX_PWM,
  COARSE_FWD_COEFF, COARSE_REVERSE_FACTOR, COARSE_TURN_COEFF_PIVOT, COARSE_TURN_COEFF_MOVING,
  COARSE_FWD_DEADZONE, COARSE_TURN_DEADZONE_PIVOT, COARSE_TURN_DEADZONE_MOVING,
  COARSE_FWD_EXPO, COARSE_TURN_EXPO,
  COARSE_ACCEL, COARSE_DECEL,
  COARSE_LEFT_TRIM, COARSE_RIGHT_TRIM
};

const ModeCfg PRECISE = {
  PRECISE_FWD_MIN_PWM, PRECISE_FWD_MAX_PWM, PRECISE_TURN_MIN_PWM, PRECISE_TURN_MAX_PWM, PRECISE_STEER_MAX_PWM,
  PRECISE_FWD_COEFF, PRECISE_REVERSE_FACTOR, PRECISE_TURN_COEFF_PIVOT, PRECISE_TURN_COEFF_MOVING,
  PRECISE_FWD_DEADZONE, PRECISE_TURN_DEADZONE_PIVOT, PRECISE_TURN_DEADZONE_MOVING,
  PRECISE_FWD_EXPO, PRECISE_TURN_EXPO,
  PRECISE_ACCEL, PRECISE_DECEL,
  PRECISE_LEFT_TRIM, PRECISE_RIGHT_TRIM
};

float clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

float axisCurve(float v, float dz, float expo) {
  float a = fabsf(v);
  if (a < dz) return 0;
  a = (a - dz) / (1.0f - dz);
  a = expo * a * a * a + (1.0f - expo) * a;
  return (v < 0) ? -a : a;
}

void computeWheelTargets(const ModeCfg &c, float f, float t, bool activeSpike, float &outL, float &outR) {
  const float THRESH = 0.02f;
  bool moving  = fabsf(f) > THRESH;
  bool turning = fabsf(t) > THRESH;
  float L = 0, R = 0;

  if (moving) {
    float mag = clamp01(fabsf(f) * c.fwdCoeff * (f < 0 ? c.reverseFactor : 1.0f));
    float base = c.fwdMin + (c.fwdMax - c.fwdMin) * mag;
    L = base;
    R = base;

    if (turning) {
      float reduce = c.steerMax * clamp01(fabsf(t) * c.turnCoeffMoving);
      bool reduceRight = ((t > 0) == (f > 0));
      if (reduceRight) R -= reduce; else L -= reduce;
      if (L < 0) L = 0;
      if (R < 0) R = 0;
    }
    if (f < 0) { L = -L; R = -R; }
  }
  else if (turning) {
    float tMax = fminf((float)c.turnMax, c.fwdMax * TURN_MAX_RATIO_OF_FWD);
    float tMin = fminf((float)c.turnMin, tMax);
    
    // Add spike boost during kick-start pulse
    if (activeSpike) {
      tMin += SPIKE_BOOST_PWM;
      if (tMin > tMax + SPIKE_BOOST_PWM) tMin = tMax + SPIKE_BOOST_PWM;
    }

    float mag = clamp01(fabsf(t) * c.turnCoeffPivot);
    float turnPwm = tMin + (tMax - tMin) * mag;
    if (t < 0) turnPwm = -turnPwm;
    L =  turnPwm;
    R = -turnPwm;
  }

  L *= c.leftTrim;
  R *= c.rightTrim;
  outL = constrain(L, -PWM_CAP, PWM_CAP);
  outR = constrain(R, -PWM_CAP, PWM_CAP);
}

float slew(float cur, float target, float accel, float decel, float dt) {
  bool speedingUp = (fabsf(target) > fabsf(cur)) && (target * cur >= 0);
  float step = (speedingUp ? accel : decel) * dt;
  if (target > cur + step) return cur + step;
  if (target < cur - step) return cur - step;
  return target;
}

const int EDGE = 250;

struct _attribute_((packed)) Packet {
  uint16_t key;
  int16_t  left;     // final PWM, -255 to 255
  int16_t  right;    // final PWM, -255 to 255
  uint8_t  flags;
  uint8_t  counter;
};

const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

Button btnG1 = {PIN_BTN_G1, false, 0};
Button btnG2 = {PIN_BTN_G2, false, 0};

int centerFwd = 2048, centerTurn = 2048;
bool calOk = false;
bool armed = false;
uint32_t centredSince = 0;
float smF = 0, smT = 0;
float curL = 0, curR = 0;
bool g1Open = false, g2Open = false;
bool preciseMode = START_IN_PRECISE_MODE;
uint8_t counter = 0;

uint32_t spikeStartTime = 0;
bool wasTurning = false;

bool     joyDown = false;
bool     joyLongDone = false;
uint32_t joyDownAt = 0;

uint8_t  blinkFlashes = 0;
bool     blinkOn = false;
uint32_t blinkNext = 0;

int readAvg(int pin) {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(pin);
  return sum / 8;
}

float normalizeAxis(int raw, int center) {
  float v;
  if (raw >= center) v = (float)(raw - center) / (float)(4095 - EDGE - center);
  else               v = (float)(raw - center) / (float)(center - EDGE);
  return constrain(v, -1.0f, 1.0f);
}

bool pressedEvent(Button &b) {
  bool p = (digitalRead(b.pin) == LOW);
  bool ev = p && !b.prev && (millis() - b.last > 200);
  if (ev) b.last = millis();
  b.prev = p;
  return ev;
}

bool calibrateStick() {
  const int N = 50;
  long sf = 0, st = 0;
  int minF = 4095, maxF = 0, minT = 4095, maxT = 0;
  for (int i = 0; i < N; i++) {
    int f = readAvg(PIN_JOY_FWD);
    int t = readAvg(PIN_JOY_TURN);
    sf += f; st += t;
    minF = min(minF, f); maxF = max(maxF, f);
    minT = min(minT, t); maxT = max(maxT, t);
    delay(10);
  }
  centerFwd  = sf / N;
  centerTurn = st / N;
  calOk = ((maxF - minF) <= STABLE_SPAN) && ((maxT - minT) <= STABLE_SPAN) &&
          (centerFwd >= CENTER_MIN) && (centerFwd <= CENTER_MAX) &&
          (centerTurn >= CENTER_MIN) && (centerTurn <= CENTER_MAX);
  return calOk;
}

void errorFlash() {
  for (int i = 0; i < 6; i++) {
    digitalWrite(PIN_LED_G1, HIGH); digitalWrite(PIN_LED_G2, HIGH);
    delay(80);
    digitalWrite(PIN_LED_G1, LOW);  digitalWrite(PIN_LED_G2, LOW);
    delay(80);
  }
}

void startModeBlink() {
  blinkFlashes = preciseMode ? 2 : 1;
  blinkOn = false;
  blinkNext = millis();
}

void recalibrate() {
  armed = false; centredSince = 0; smF = 0; smT = 0;
  if (!calibrateStick()) errorFlash();
}

void handleJoyButton(uint32_t now) {
  bool p = (digitalRead(PIN_JOY_BTN) == LOW);
  if (p && !joyDown) {
    joyDown = true; joyLongDone = false; joyDownAt = now;
  } else if (p && joyDown) {
    if (!joyLongDone && (now - joyDownAt) >= RECAL_HOLD_MS) {
      joyLongDone = true; recalibrate();
    }
  } else if (!p && joyDown) {
    joyDown = false;
    uint32_t held = now - joyDownAt;
    if (!joyLongDone && held >= 50 && held < RECAL_HOLD_MS) {
      preciseMode = !preciseMode;
      smF = 0; smT = 0;
      startModeBlink();
    }
  }
}

void updateLeds(uint32_t now) {
  if (blinkFlashes > 0) {
    if (now >= blinkNext) {
      blinkOn = !blinkOn;
      if (!blinkOn) blinkFlashes--;
      blinkNext = now + 120;
    }
    digitalWrite(PIN_LED_G1, blinkOn ? HIGH : LOW);
    digitalWrite(PIN_LED_G2, blinkOn ? HIGH : LOW);
    if (blinkFlashes > 0) return;
  }
  if (!armed) {
    bool phase = ((now / 250) % 2) == 0;
    digitalWrite(PIN_LED_G1, phase ? HIGH : LOW);
    digitalWrite(PIN_LED_G2, phase ? LOW : HIGH);
    return;
  }
  digitalWrite(PIN_LED_G1, (LED_ON_WHEN_OPEN ? g1Open : !g1Open) ? HIGH : LOW);
  digitalWrite(PIN_LED_G2, (LED_ON_WHEN_OPEN ? g2Open : !g2Open) ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_BTN_G1, INPUT_PULLUP);
  pinMode(PIN_BTN_G2, INPUT_PULLUP);
  pinMode(PIN_JOY_BTN, INPUT_PULLUP);
  pinMode(PIN_LED_G1, OUTPUT);
  pinMode(PIN_LED_G2, OUTPUT);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  delay(500);
  for (int attempt = 0; attempt < 3; attempt++) {
    if (calibrateStick()) break;
    errorFlash();
    delay(300);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) while (true) delay(1000);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  startModeBlink();
}

void loop() {
  static uint32_t lastSend = 0;
  uint32_t now = millis();
  if (now - lastSend < 20) return;
  float dt = (now - lastSend) / 1000.0f;
  if (dt > 0.05f) dt = 0.05f;
  lastSend = now;

  if (pressedEvent(btnG1)) g1Open = !g1Open;
  if (pressedEvent(btnG2)) g2Open = !g2Open;
  handleJoyButton(now);

  const ModeCfg &c = preciseMode ? PRECISE : COARSE;

  float rawF = normalizeAxis(readAvg(PIN_JOY_FWD),  centerFwd);
  float rawT = normalizeAxis(readAvg(PIN_JOY_TURN), centerTurn);

  bool centred = (fabsf(rawF) < ARM_STICK_ZONE) && (fabsf(rawT) < ARM_STICK_ZONE);
  if (!armed) {
    if (calOk && centred) {
      if (centredSince == 0) centredSince = now;
      else if (now - centredSince >= ARM_MS) armed = true;
    } else {
      centredSince = 0;
    }
  }

  updateLeds(now);

  float f = axisCurve(rawF, c.fwdDeadzone, c.fwdExpo);
  float t = axisCurve(rawT, (f != 0) ? c.turnDzMoving : c.turnDzPivot, c.turnExpo);
  if (INVERT_FORWARD) f = -f;
  if (INVERT_TURN)    t = -t;
  if (!armed) { f = 0; t = 0; }

  smF += STICK_SMOOTHING * (f - smF);
  smT += TURN_SMOOTHING  * (t - smT);
  if (fabsf(smF) < 0.02f) smF = 0;
  if (fabsf(smT) < 0.02f) smT = 0;

  // Stiction Kick Trigger
  bool isTurning = fabsf(smT) > 0.02f;
  if (isTurning && !wasTurning) {
    spikeStartTime = now;
  }
  wasTurning = isTurning;
  bool activeSpike = isTurning && (now - spikeStartTime < SPIKE_DURATION_MS);

  float tgtL, tgtR;
  computeWheelTargets(c, smF, smT, activeSpike, tgtL, tgtR);

  curL = slew(curL, tgtL, c.accel, c.decel, dt);
  curR = slew(curR, tgtR, c.accel, c.decel, dt);

  Packet p;
  p.key     = TEAM_KEY;
  p.left    = (int16_t)lroundf(curL);
  p.right   = (int16_t)lroundf(curR);
  p.flags   = (g1Open ? 1 : 0) | (g2Open ? 2 : 0) | (preciseMode ? 4 : 0);
  p.counter = counter++;

  esp_now_send(BROADCAST, (uint8_t *)&p, sizeof(p));

  static uint32_t lastPrint = 0;
  if (now - lastPrint > 500) {
    lastPrint = now;
    Serial.printf("%s %s  PWM L=%d R=%d  stick fwd=%.2f turn=%.2f%s\n",
                  armed ? "ARMED" : "NOT-ARMED", preciseMode ? "PRECISE" : "COARSE",
                  p.left, p.right, rawF, rawT, activeSpike ? " [KICK]" : "");
  }
}
