#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <NimBLEDevice.h>

// ================== YOUR HARDWARE ==================
const int SERVO_ROLL_PIN  = 6;   // roll servo signal pin
const int SERVO_PITCH_PIN = 7;   // pitch servo signal pin

// Nano ESP32 I2C mapping (works for you)
const int I2C_SDA_PIN = A4;
const int I2C_SCL_PIN = A5;

// ================== SERVO LIMITS (+/- 15 ONLY) ==================
const int SERVO_CENTER = 90;
const int SERVO_DELTA_MAX = 15;            // +/- 15 degrees
const int SERVO_ROLL_MIN  = SERVO_CENTER - SERVO_DELTA_MAX;   // 75
const int SERVO_ROLL_MAX  = SERVO_CENTER + SERVO_DELTA_MAX;   // 105
const int SERVO_PITCH_MIN = SERVO_CENTER - SERVO_DELTA_MAX;   // 75
const int SERVO_PITCH_MAX = SERVO_CENTER + SERVO_DELTA_MAX;   // 105

// Map tilt to servo travel
float MAX_TILT_DEG = 30.0f;   // tilt range that maps to full +/-15 servo travel

// Flip if directions are backwards
bool INVERT_ROLL  = false;
bool INVERT_PITCH = false;

// Smoothing (0..1): lower = smoother, higher = more responsive
float SMOOTH_ALPHA = 0.15f;

// Safer arming
const char* ARM_PIN = "1234";
bool REQUIRE_CAL_BEFORE_ARM = false; // no CAL required; we will NOT use CAL in this version

// Optional telemetry (OFF by default so responses are easy to read)
bool telemetryOn = false;
uint32_t telemetryIntervalMs = 250;
uint32_t controlIntervalMs = 20; // 50Hz servo update

// ================== BLE NUS UUIDs ==================
static NimBLEUUID NUS_SERVICE("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID NUS_RX_CHAR("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID NUS_TX_CHAR("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

// ================== GLOBALS ==================
Adafruit_MPU6050 mpu;
Servo servoRoll, servoPitch;

NimBLECharacteristic* txChar = nullptr;
volatile bool deviceConnected = false;

bool imuOk = false;
bool armed = false;

// We keep offsets but do not expose CAL here; you can enable auto-zero later
bool calibrated = false;
float pitchOffsetDeg = 0.0f;
float rollOffsetDeg  = 0.0f;

float pitchDeg = 0.0f, rollDeg = 0.0f;
int rollServoCmd = 90, pitchServoCmd = 90;

float rollServoSmoothed  = 90.0f;
float pitchServoSmoothed = 90.0f;

// ================== HELPERS ==================
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
  char buf[200];
  snprintf(buf, sizeof(buf), "RESP %s\n", msg);
  sendLine(buf);
}

static void setArmed(bool on) {
  armed = on;
  if (!armed) {
    servoRoll.write(SERVO_CENTER);
    servoPitch.write(SERVO_CENTER);
  }
}

static void computePitchRollDeg(const sensors_event_t& a, float& pitchOut, float& rollOut) {
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
  float roll  = atan2( ay, az ) * 180.0f / PI;

  pitchOut = pitch;
  rollOut  = roll;
}

static void sendStatus() {
  char buf[240];
  snprintf(buf, sizeof(buf),
           "RESP STATUS t=%lu imu=%d armed=%d pitch=%.1f roll=%.1f sRoll=%d sPitch=%d limits=[%d..%d]\n",
           (unsigned long)millis(),
           imuOk ? 1 : 0,
           armed ? 1 : 0,
           pitchDeg, rollDeg,
           rollServoCmd, pitchServoCmd,
           SERVO_ROLL_MIN, SERVO_ROLL_MAX);
  sendLine(buf);
}

// ================== BLE CALLBACKS ==================
class ServerCallbacks : public NimBLEServerCallbacks {
public:
  void onConnect(NimBLEServer* s, NimBLEConnInfo& ci) {
    (void)s; (void)ci;
    deviceConnected = true;
    sendResp("OK CONNECTED");
    sendResp("Commands: ARM 1234 | DISARM | STATUS | DEBUG ON | DEBUG OFF");
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& ci, int reason) {
    (void)s; (void)ci; (void)reason;
    deviceConnected = false;
    setArmed(false); // auto-disarm for safety
  }

  // Older signature compatibility
  void onConnect(NimBLEServer* s) {
    (void)s;
    deviceConnected = true;
    sendResp("OK CONNECTED");
    sendResp("Commands: ARM 1234 | DISARM | STATUS | DEBUG ON | DEBUG OFF");
  }
  void onDisconnect(NimBLEServer* s) {
    (void)s;
    deviceConnected = false;
    setArmed(false);
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& ci) { (void)ci; handleRx(c); }
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

    if (v == "STATUS") { sendStatus(); return; }

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
      if (REQUIRE_CAL_BEFORE_ARM && !calibrated) { sendResp("ERR NOT CALIBRATED"); return; }
      setArmed(true);
      sendResp("OK ARMED");
      return;
    }

    if (v == "DEBUG ON")  { telemetryOn = true;  sendResp("OK DEBUG ON");  return; }
    if (v == "DEBUG OFF") { telemetryOn = false; sendResp("OK DEBUG OFF"); return; }

    sendResp("ERR unknown cmd");
  }
};

// ================== SETUP/LOOP ==================
void setup() {
  // I2C + IMU
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

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
  servoRoll.write(SERVO_CENTER);
  servoPitch.write(SERVO_CENTER);

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

  // ---- Control loop ----
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

      // Map tilt to +/-15 degrees
      float pitchTarget = (float)SERVO_CENTER + (p / MAX_TILT_DEG) * (float)SERVO_DELTA_MAX;
      float rollTarget  = (float)SERVO_CENTER + (r / MAX_TILT_DEG) * (float)SERVO_DELTA_MAX;

      // Clamp to +/-15 range
      pitchTarget = clampf(pitchTarget, (float)SERVO_PITCH_MIN, (float)SERVO_PITCH_MAX);
      rollTarget  = clampf(rollTarget,  (float)SERVO_ROLL_MIN,  (float)SERVO_ROLL_MAX);

      // Smooth
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
      rollServoCmd = SERVO_CENTER;
      pitchServoCmd = SERVO_CENTER;
    }
  }

  // ---- Optional telemetry (quiet by default) ----
  if (telemetryOn && deviceConnected && (now - lastTelem >= telemetryIntervalMs)) {
    lastTelem = now;
    char buf[220];
    snprintf(buf, sizeof(buf),
             "DATA t=%lu armed=%d pitch=%.1f roll=%.1f sRoll=%d sPitch=%d\n",
             (unsigned long)now,
             armed ? 1 : 0,
             pitchDeg, rollDeg,
             rollServoCmd, pitchServoCmd);
    sendLine(buf);
  }

  delay(2);
}
