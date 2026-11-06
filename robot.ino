// ============================================================
//  REC Group 1 - ROBOT
//  Board: ESP32 DevKit V1 (30-pin) + TB6612FNG + 2x SG90 servo
//
//  This code is intentionally simple. The CONTROLLER works out the final PWM for
//  each wheel (speed ranges, dead zones, turning damping, easing, drift trim...).
//  The robot just carries those numbers out. Tune driving in the controller code.
//
//  Arduino IDE: Board = "ESP32 Dev Module". No extra libraries needed.
// ============================================================
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ---------- 1. MUST BE THE SAME IN THE CONTROLLER CODE ----------
const uint16_t TEAM_KEY     = 0x5A27;
const uint8_t  WIFI_CHANNEL = 11;

// ---------- 2. PINS ----------
const int PIN_PWMA = 25;
const int PIN_AIN1 = 33;
const int PIN_AIN2 = 32;
const int PIN_PWMB = 26;
const int PIN_BIN1 = 19;
const int PIN_BIN2 = 18;
const int PIN_STBY = 13;
const int PIN_SERVO1 = 22;   // Front Gripper 1
const int PIN_SERVO2 = 23;   // Front Gripper 2

// ---------- 3. WIRING (set once, then leave) ----------
const bool SWAP_MOTORS = false;  // true if left and right wheels are the wrong way round
const bool INVERT_A    = false;  // true if wheel A spins backward when it should go forward
const bool INVERT_B    = true;   // true if wheel B spins backward when it should go forward

// Safety cap, whatever the controller asks for (protects the TT motors on 7.5 V).
// 204 / 255 x 7.5 V = about 6 V average. Not a tuning knob: tune speed in the controller.
const int ROBOT_PWM_LIMIT = 204;
const uint32_t FAILSAFE_MS = 300;  // stop the wheels if no signal for this long

// ---------- 4. GRIPPER SETTINGS ----------
// Angles in degrees (0 to 180). If a servo buzzes at the end of its travel, reduce the angle.
const float G1_CLOSED = 120;
const float G1_OPEN   = 0;
const float G2_CLOSED = 110;
const float G2_OPEN   = 0;
const float SERVO_SPEED = 120;    // degrees per second (lower = gentler, less current)

// ============================================================
//  You normally do not need to change anything below this line
// ============================================================
// Must match the controller exactly.
struct __attribute__((packed)) Packet {
  uint16_t key;
  int16_t  left;    // final PWM, -255 to 255 (0 = brake)
  int16_t  right;
  uint8_t  flags;   // bit0 = gripper 1 open, bit1 = gripper 2 open, bit2 = precise mode
  uint8_t  counter;
};

const int CH_A = 0, CH_B = 1, CH_S1 = 2, CH_S2 = 3;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch, freq, bits) ledcAttach(pin, freq, bits)
  #define PWM_WRITE(pin, ch, duty)        ledcWrite(pin, duty)
#else
  #define PWM_ATTACH(pin, ch, freq, bits) do { ledcSetup(ch, freq, bits); ledcAttachPin(pin, ch); } while (0)
  #define PWM_WRITE(pin, ch, duty)        ledcWrite(ch, duty)
#endif

volatile int16_t  rxLeft = 0, rxRight = 0;
volatile uint8_t  rxFlags = 0;
volatile uint32_t rxTime = 0;

float g1Angle = G1_CLOSED, g2Angle = G2_CLOSED;

void handlePacket(const uint8_t *data, int len) {
  if (len != (int)sizeof(Packet)) return;
  Packet p;
  memcpy(&p, data, sizeof(p));
  if (p.key != TEAM_KEY) return;  // ignore other teams
  rxLeft  = p.left;
  rxRight = p.right;
  rxFlags = p.flags;
  rxTime  = millis();
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) { handlePacket(data, len); }
#else
void onReceive(const uint8_t *mac, const uint8_t *data, int len) { handlePacket(data, len); }
#endif

// Drive one motor with a signed PWM value. 0 = short brake (stops firmly).
void driveDuty(int in1, int in2, int pwmPin, int ch, int duty, bool invert) {
  if (duty >  ROBOT_PWM_LIMIT) duty =  ROBOT_PWM_LIMIT;
  if (duty < -ROBOT_PWM_LIMIT) duty = -ROBOT_PWM_LIMIT;
  if (invert) duty = -duty;
  if (duty == 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    PWM_WRITE(pwmPin, ch, 255);
    return;
  }
  bool fwd = duty > 0;
  digitalWrite(in1, fwd ? HIGH : LOW);
  digitalWrite(in2, fwd ? LOW : HIGH);
  PWM_WRITE(pwmPin, ch, abs(duty));
}

void servoWrite(int pin, int ch, float angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  float us = 500.0f + (angle / 180.0f) * 1900.0f;       // SG90: about 0.5 ms to 2.4 ms
  uint32_t duty = (uint32_t)(us / 20000.0f * 16383.0f);  // 50 Hz, 14-bit
  PWM_WRITE(pin, ch, duty);
}

float moveToward(float cur, float target, float maxStep) {
  if (target > cur + maxStep) return cur + maxStep;
  if (target < cur - maxStep) return cur - maxStep;
  return target;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);  // driver ON (nothing moves if this stays LOW)

  PWM_ATTACH(PIN_PWMA, CH_A, 20000, 8);
  PWM_ATTACH(PIN_PWMB, CH_B, 20000, 8);
  PWM_ATTACH(PIN_SERVO1, CH_S1, 50, 14);
  PWM_ATTACH(PIN_SERVO2, CH_S2, 50, 14);

  driveDuty(PIN_AIN1, PIN_AIN2, PIN_PWMA, CH_A, 0, false);
  driveDuty(PIN_BIN1, PIN_BIN2, PIN_PWMB, CH_B, 0, false);
  servoWrite(PIN_SERVO1, CH_S1, g1Angle);  // start with grippers closed (compact)
  servoWrite(PIN_SERVO2, CH_S2, g2Angle);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW failed to start");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onReceive);
  Serial.println("Robot ready");
}

void loop() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 10) return;       // run 100 times per second
  float dt = (now - last) / 1000.0f;
  last = now;

  bool linkOk = (rxTime != 0) && ((now - rxTime) < FAILSAFE_MS);
  uint8_t flags = rxFlags;

  // ---- wheels: carry out exactly what the controller sent ----
  int left  = linkOk ? rxLeft  : 0;   // lost signal: stop right away
  int right = linkOk ? rxRight : 0;
  int a = SWAP_MOTORS ? right : left;  // motor A
  int b = SWAP_MOTORS ? left : right;  // motor B
  driveDuty(PIN_AIN1, PIN_AIN2, PIN_PWMA, CH_A, a, INVERT_A);
  driveDuty(PIN_BIN1, PIN_BIN2, PIN_PWMB, CH_B, b, INVERT_B);

  // ---- grippers (keep their last position if the signal is lost) ----
  if (linkOk) {
    float t1 = (flags & 1) ? G1_OPEN : G1_CLOSED;
    float t2 = (flags & 2) ? G2_OPEN : G2_CLOSED;
    float step = SERVO_SPEED * dt;
    g1Angle = moveToward(g1Angle, t1, step);
    g2Angle = moveToward(g2Angle, t2, step);
    servoWrite(PIN_SERVO1, CH_S1, g1Angle);
    servoWrite(PIN_SERVO2, CH_S2, g2Angle);
  }

  static uint32_t lastPrint = 0;
  if (now - lastPrint > 500) {
    lastPrint = now;
    Serial.printf("link=%d %s PWM L=%d R=%d G1=%.0f G2=%.0f\n",
                  linkOk, (flags & 4) ? "PRECISE" : "COARSE", left, right, g1Angle, g2Angle);
  }
}
