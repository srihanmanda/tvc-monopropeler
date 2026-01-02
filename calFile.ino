#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <NimBLEDevice.h>

// ================== USER SETTINGS ==================

// Your servo pins (as requested)
const int SERVO_ROLL_PIN  = 6;  // Roll servo signal
const int SERVO_PITCH_PIN = 7;  // Pitch servo signal

// I2C pins for Nano ESP32 headers (your working scan found 0x68 on these)
const int I2C_SDA_PIN = A4;
const int I2C_SCL_PIN = A5;

// Servo safety limits (degrees)
int SERVO_ROLL_MIN  = 20;
int SERVO_ROLL_MAX  = 160;
int SERVO_PITCH_MIN = 20;
int SERVO_PITCH_MAX = 160;

// Tilt mapping
float MAX_TILT_DEG = 30.0f;     // clamp IMU tilt used for control
float SERVO_GAIN   = 2.0f;      // deg servo per deg tilt (2.0 => 30deg tilt -> 60deg servo change)

// Invert directions if servo moves opposite of expected
bool INVERT_ROLL  = false;
bool INVERT_PITCH = false;

// Smoothing (0..1). Higher = more responsive, lower = smoother.
float SMOOTH_ALPHA = 0.15f;

// Require calibration before arming
bool REQUIRE_CAL_BEFORE_ARM = true;

// Arm PIN (simple safety gate)
const char* ARM_PIN = "1234";

// Control loop rate (servo update)
uint32_t controlIntervalMs = 20;   // 50 Hz control

// Telemetry rate (BLE noisy stream)
uint32_t telemetryIntervalMs = 200; // slower by default

// ====================================================

// BLE NUS UUIDs (Nordic UART Service)
static NimBLEUUID NUS_SERVICE("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID NUS_RX_CHAR("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // phone writes here
static NimBLEUUID NUS_TX_CHAR("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // notify to phone

Adafruit_MPU6050 mpu;
Servo servoRoll, servoPitch;

NimBLECharacteristic* txChar = nullptr;
volatile bool deviceConnected = false;

bool imuOk = false;

bool armed = false;
bool calibrated = false;
float pitchOffsetDeg = 0.0f;
float rollOffsetDeg  = 0.0f;

// Telemetry/Debug stream (OFF by default to keep command responses readable)
bool telemetryOn = false;

// Smoothed servo outputs
float rollServoSmoothed  = 90.0f;
float pitchServoSmoothed = 90.0f;

// Latest computed state (used for STATUS / telemetry)
float pitchDeg = 0.0f, rollDeg = 0.0f;
int rollServoCmd = 90, pitchServoCmd = 90;

static int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void sendLine(const char* s) {
  if (deviceConnected && txChar) {
    txChar->setValue((uint8_t*)s, strlen(s));
    txChar->notify();
  }
}

static void sendResp(const char* msg) {
  char buf[160];
  snprintf(buf, sizeof(buf), "RESP %s\n", msg);
  sendLine(buf);
}

static void sendStatusLine() {
  char buf[220];
  snprintf(buf, sizeof(buf),
           "RESP STATUS t=%lu imu=%d cal=%d armed=%d pitch=%.1f roll=%.1f sRoll=%d sPitch=%d\n",
           (unsigned long)millis(),
           imuOk ? 1 : 0,
           calibrated ? 1 : 0,
           armed ? 1 : 0,
           pitchDeg, rollDeg,
           rollServoCmd, pitchServoCmd);
  sendLine(buf);
}

static void setArmed(bool on) {
  armed = on;
  if (!armed) {
    // Safe: center servos
    servoRoll.write(90);
    servoPitch.write(90);
  }
}

// Compute accel-only pitch/roll (good for hand testing)
static void computePitchRollDeg(const sensors_event_t& a, float& pitchOut, float& rollOut) {
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
  float roll  = atan2( ay, az ) * 180.0f / PI;

  pitchOut = pitch;
  rollOut  = roll;
}

static void doCalibration() {
  if (!imuOk) {
    sendResp("ERR IMU not OK");
    return;
  }

  sendResp("CAL START (hold level + still)");

  const int N = 200;
  float pitchSum = 0.0f;
  float rollSum  = 0.0f;

  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    float p, r;
    computePitchRollDeg(a, p, r);
    pitchSum += p;
    rollSum  += r;

    delay(5);
  }

  pitchOffsetDeg = pitchSum / (float)N;
  rollOffsetDeg  = rollSum  / (float)N;
  calibrated = true;

  char msg[120];
  snprintf(msg, sizeof(msg), "OK CAL pitchOffset=%.2f rollOffset=%.2f", pitchOffsetDeg, rollOffsetDeg);
  sendResp(msg);
}

// ---------- BLE callbacks ----------
class ServerCallbacks : public NimBLEServerCallbacks {
public:
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
    (void)pServer; (void)connInfo;
    deviceConnected = true;
    sendResp("OK CONNECTED");
    sendResp("Commands: CAL | ARM 1234 | DISARM | STATUS | DEBUG ON/OFF | LIMITS rMin rMax pMin pMax");
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    (void)pServer; (void)connInfo; (void)reason;
    deviceConnected = false;
    setArmed(false); // auto-disarm for safety
  }

  // Older signature compatibility
  void onConnect(NimBLEServer* pServer) {
    (void)pServer;
    deviceConnected = true;
    sendResp("OK CONNECTED");
    sendResp("Commands: CAL | ARM 1234 | DISARM | STATUS | DEBUG ON/OFF | LIMITS rMin rMax pMin pMax");
  }
  void onDisconnect(NimBLEServer* pServer) {
    (void)pServer;
    deviceConnected = false;
    setArmed(false);
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) { (void)connInfo; handleRx(c); }
  void onWrite(NimBLECharacteristic* c) { handleRx(c); }

private:
  void handleRx(NimBLECharacteristic* c) {
    std::string v = c->getValue();
    if (v.empty()) return;
    if (v.size() > 64) { sendResp("ERR cmd too long"); return; }

    for (char ch : v) {
      if (ch < 0x20 || ch > 0x7E) { sendResp("ERR bad chars"); return; }
    }
    while (!v.empty() && (v.back() == '\r' || v.back() == '\n')) v.pop_back();

    if (v == "CAL") {
      doCalibration();
      return;
    }

    if (v == "STATUS") {
      sendStatusLine();
      return;
    }

    if (v == "DISARM") {
      setArmed(false);
      sendResp("OK DISARMED");
      return;
    }

    if (v == "ARM") {
      sendResp("ERR use: ARM 1234");
      return;
    }

    if (v.rfind("ARM ", 0) == 0) {
      std::string pin = v.substr(4);
      if (pin != ARM_PIN) { sendResp("ERR BAD PIN"); return; }
      if (REQUIRE_CAL_BEFORE_ARM && !calibrated) { sendResp("ERR NOT CALIBRATED (send CAL first)"); return; }
      setArmed(true);
      sendResp("OK ARMED");
      return;
    }

    if (v == "DEBUG ON") {
      telemetryOn = true;
      sendResp("OK DEBUG ON");
      return;
    }
    if (v == "DEBUG OFF") {
      telemetryOn = false;
      sendResp("OK DEBUG OFF");
      return;
    }

    if (v.rfind("LIMITS ", 0) == 0) {
      int rMin, rMax, pMin, pMax;
      if (sscanf(v.c_str(), "LIMITS %d %d %d %d", &rMin, &rMax, &pMin, &pMax) != 4) {
        sendResp("ERR LIMITS format");
        return;
      }
      if (rMin < 0 || rMax > 180 || pMin < 0 || pMax > 180 || rMin >= rMax || pMin >= pMax) {
        sendResp("ERR LIMITS range");
        return;
      }
      SERVO_ROLL_MIN = rMin; SERVO_ROLL_MAX = rMax;
      SERVO_PITCH_MIN = pMin; SERVO_PITCH_MAX = pMax;
      sendResp("OK LIMITS");
      return;
    }

    sendResp("ERR unknown cmd");
  }
};

void setup() {
  // I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  // IMU
  imuOk = mpu.begin();
  if (imuOk) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // Servos
  servoRoll.setPeriodHertz(50);
  servoPitch.setPeriodHertz(50);
  servoRoll.attach(SERVO_ROLL_PIN, 500, 2400);
  servoPitch.attach(SERVO_PITCH_PIN, 500, 2400);
  servoRoll.write(90);
  servoPitch.write(90);

  // BLE
  NimBLEDevice::init("NanoESP32-BLE-UART");
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService* svc = server->createService(NUS_SERVICE);

  txChar = svc->createCharacteristic(NUS_TX_CHAR, NIMBLE_PROPERTY::NOTIFY);

  NimBLECharacteristic* rxChar = svc->createCharacteristic(
    NUS_RX_CHAR,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rxChar->setCallbacks(new RxCallbacks());

  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->start();
}

void loop() {
  static uint32_t lastControl = 0;
  static uint32_t lastTelem = 0;

  uint32_t now = millis();

  // ---------- Control update ----------
  if (now - lastControl >= controlIntervalMs) {
    lastControl = now;

    if (imuOk) {
      sensors_event_t a, g, t;
      mpu.getEvent(&a, &g, &t);

      float pRaw, rRaw;
      computePitchRollDeg(a, pRaw, rRaw);

      pitchDeg = pRaw - (calibrated ? pitchOffsetDeg : 0.0f);
      rollDeg  = rRaw - (calibrated ? rollOffsetDeg  : 0.0f);

      float p = clampf(pitchDeg, -MAX_TILT_DEG, MAX_TILT_DEG);
      float r = clampf(rollDeg,  -MAX_TILT_DEG, MAX_TILT_DEG);

      if (INVERT_PITCH) p = -p;
      if (INVERT_ROLL)  r = -r;

      float rollTarget  = 90.0f + (r * SERVO_GAIN);
      float pitchTarget = 90.0f + (p * SERVO_GAIN);

      rollTarget  = clampf(rollTarget,  (float)SERVO_ROLL_MIN,  (float)SERVO_ROLL_MAX);
      pitchTarget = clampf(pitchTarget, (float)SERVO_PITCH_MIN, (float)SERVO_PITCH_MAX);

      rollServoSmoothed  = rollServoSmoothed  + SMOOTH_ALPHA * (rollTarget  - rollServoSmoothed);
      pitchServoSmoothed = pitchServoSmoothed + SMOOTH_ALPHA * (pitchTarget - pitchServoSmoothed);

      rollServoCmd  = clampi((int)(rollServoSmoothed + 0.5f),  SERVO_ROLL_MIN,  SERVO_ROLL_MAX);
      pitchServoCmd = clampi((int)(pitchServoSmoothed + 0.5f), SERVO_PITCH_MIN, SERVO_PITCH_MAX);

      if (armed) {
        servoRoll.write(rollServoCmd);
        servoPitch.write(pitchServoCmd);
      }
    } else {
      pitchDeg = 0.0f; rollDeg = 0.0f;
      rollServoCmd = 90; pitchServoCmd = 90;
    }
  }

  // ---------- Telemetry (optional / less noisy) ----------
  if (telemetryOn && deviceConnected && (now - lastTelem >= telemetryIntervalMs)) {
    lastTelem = now;
    char buf[220];
    snprintf(buf, sizeof(buf),
             "DATA t=%lu imu=%d cal=%d armed=%d pitch=%.1f roll=%.1f sRoll=%d sPitch=%d\n",
             (unsigned long)now,
             imuOk ? 1 : 0,
             calibrated ? 1 : 0,
             armed ? 1 : 0,
             pitchDeg, rollDeg,
             rollServoCmd, pitchServoCmd);
    sendLine(buf);
  }

  delay(2);
}
