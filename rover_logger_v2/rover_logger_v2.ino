/*
  rover_logger_v2.ino  -  ESP32 + 4x VL53L0X + MPU6050 data logger for the maze rover

  What changed from the first sketch (see analysis/README.md for why):
    * ToF sensors run in CONTINUOUS mode and are polled without blocking
      -> ~28 Hz per sensor instead of 4-5 Hz for the whole loop
    * IMU is sampled at 200 Hz in its own timed loop; gyro bias is calibrated at start-up
      and yaw is integrated ON the ESP32 (yaw_deg, counter-clockwise positive seen from above)
      -> short turns are no longer lost between log lines
    * gyro_z in the log is the MEAN over the log interval (not one instant), printed with 4 decimals,
      plus gz_peak = largest |rate| inside the interval
    * a range is -1 whenever the sensor status is not "valid" (no more 8191 sneaking through),
      and the raw status of each sensor is logged too (0 = valid, 1 sigma, 2 signal, 3 min range, 4 out of range)
    * lines starting with '#' are comments / events; the CSV header is printed after calibration
    * optional WiFi: set USE_WIFI 1 to stream the same lines over UDP (for remote-driven runs)

  CSV columns (first 11 are identical to the old sketch, so the old analysis still works):
    timestamp_ms,front_mm,right_mm,left_mm,back_mm,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,
    yaw_deg,gz_peak,st_f,st_r,st_l,st_b

  Commands (Serial monitor or UDP, single character):
    c = re-calibrate gyro (robot must be still ~2 s)   z = zero yaw
    m = event marker (prints "#MARK,<ms>,<n>")          H = print header again
    h = hello (WiFi: tells the ESP32 where to send data)

  Libraries: Adafruit_VL53L0X (>= 1.2.0), Adafruit MPU6050, Adafruit Unified Sensor. Board: ESP32 Dev Module.
*/
#include <Wire.h>
#include <stdarg.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ------------------------------------------------------------------ user settings
#define USE_WIFI 0                         // 1 = also stream over WiFi UDP
#if USE_WIFI
  #include <WiFi.h>
  #include <WiFiUdp.h>
  const char* WIFI_SSID = "YOUR_WIFI_NAME";
  const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
  const uint16_t UDP_PORT = 4210;          // laptop listens here (record_robot.py --udp 4210)
#endif

#define SDA_PIN 21
#define SCL_PIN 22
#define XSHUT_FRONT 32
#define XSHUT_RIGHT 33
#define XSHUT_LEFT  27
#define XSHUT_BACK  26

const uint32_t I2C_HZ          = 400000;   // drop to 100000 if sensors drop out on long wires
const uint32_t IMU_PERIOD_US   = 5000;     // 200 Hz IMU / yaw integration
const uint32_t LOG_PERIOD_MS   = 40;       // 25 lines per second
const uint32_t TOF_BUDGET_US   = 33000;    // VL53L0X timing budget (33 ms = default accuracy)
const uint16_t TOF_PERIOD_MS   = 35;       // continuous-mode inter-measurement period
const uint16_t TOF_MAX_MM      = 2000;     // anything above is treated as invalid
const uint16_t CAL_SAMPLES     = 400;      // 2 s at 200 Hz
const float    CAL_MAX_STD     = 0.02f;    // rad/s; more wobble than this = robot was moving

// ------------------------------------------------------------------ objects
struct Tof {
  const char* name; uint8_t xshut; uint8_t addr;
  Adafruit_VL53L0X dev;
  bool ok; int16_t mm; uint8_t status;
  Tof(const char* n, uint8_t x, uint8_t a) : name(n), xshut(x), addr(a), ok(false), mm(-1), status(255) {}
};
Tof tof[4] = { {"front", XSHUT_FRONT, 0x30}, {"right", XSHUT_RIGHT, 0x31},
               {"left",  XSHUT_LEFT,  0x32}, {"back",  XSHUT_BACK,  0x33} };
Adafruit_MPU6050 mpu;

#if USE_WIFI
WiFiUDP udp;
IPAddress peer; bool havePeer = false;
#endif

// IMU state
float biasX = 0, biasY = 0, biasZ = 0;
float zSign = -1.0f;              // +1 if the IMU z axis points up, -1 if down (set by calibration)
double yawRad = 0;
float prevRate = 0;
uint32_t lastImuUs = 0, lastLogMs = 0;
// accumulators for one log interval
double sAx = 0, sAy = 0, sAz = 0, sGx = 0, sGy = 0, sGz = 0; uint16_t nAcc = 0; float gzPeak = 0;
uint32_t markCount = 0;

const char* HEADER = "timestamp_ms,front_mm,right_mm,left_mm,back_mm,acc_x,acc_y,acc_z,"
                     "gyro_x,gyro_y,gyro_z,yaw_deg,gz_peak,st_f,st_r,st_l,st_b";

// ------------------------------------------------------------------ output
void emit(const char* line) {
  Serial.println(line);
#if USE_WIFI
  if (WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(havePeer ? peer : IPAddress(255, 255, 255, 255), UDP_PORT);
    udp.write((const uint8_t*)line, strlen(line));
    udp.write((const uint8_t*)"\n", 1);
    udp.endPacket();
  }
#endif
}
void emitf(const char* fmt, ...) {
  char buf[200]; va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap); emit(buf);
}

// ------------------------------------------------------------------ setup helpers
void initTof() {
  for (auto& s : tof) { pinMode(s.xshut, OUTPUT); digitalWrite(s.xshut, LOW); }
  delay(20);
  for (auto& s : tof) {
    pinMode(s.xshut, INPUT);                 // release this sensor (module has its own pull-up)
    delay(20);
    s.ok = s.dev.begin(s.addr, false, &Wire);
    if (!s.ok) { emitf("#ERR,tof_%s_not_found", s.name); continue; }
    s.dev.setMeasurementTimingBudgetMicroSeconds(TOF_BUDGET_US);
    s.dev.startRangeContinuous(TOF_PERIOD_MS);
    emitf("#OK,tof_%s,addr=0x%02X", s.name, s.addr);
  }
}

void pollTof() {
  for (auto& s : tof) {
    if (!s.ok || !s.dev.isRangeComplete()) continue;
    uint16_t r = s.dev.readRangeResult();
    s.status = s.dev.readRangeStatus();
    s.mm = (s.status == 0 && r > 0 && r < TOF_MAX_MM) ? (int16_t)r : -1;
  }
}

bool calibrate() {
  emit("#CALIBRATING,keep_the_robot_still");
  for (int attempt = 0; attempt < 5; attempt++) {
    double sx = 0, sy = 0, sz = 0, sz2 = 0, saz = 0;
    for (int i = 0; i < CAL_SAMPLES; i++) {
      sensors_event_t a, g, t; mpu.getEvent(&a, &g, &t);
      sx += g.gyro.x; sy += g.gyro.y; sz += g.gyro.z; sz2 += (double)g.gyro.z * g.gyro.z; saz += a.acceleration.z;
      delayMicroseconds(IMU_PERIOD_US);
    }
    double mz = sz / CAL_SAMPLES, sdz = sqrt(fmax(0.0, sz2 / CAL_SAMPLES - mz * mz));
    if (sdz < CAL_MAX_STD) {
      biasX = sx / CAL_SAMPLES; biasY = sy / CAL_SAMPLES; biasZ = mz;
      zSign = (saz / CAL_SAMPLES) > 0 ? 1.0f : -1.0f;
      emitf("#CAL,bias_x=%.5f,bias_y=%.5f,bias_z=%.5f,std_z=%.5f,acc_z=%.3f,z_axis=%s",
            biasX, biasY, biasZ, sdz, saz / CAL_SAMPLES, zSign > 0 ? "up" : "down");
      return true;
    }
    emitf("#WARN,robot_moved_during_calibration,std_z=%.4f,retrying", sdz);
  }
  emit("#WARN,calibration_failed_using_last_bias");
  return false;
}

void resetAccumulators() { sAx = sAy = sAz = sGx = sGy = sGz = 0; nAcc = 0; gzPeak = 0; }

void handleCommand(char c) {
  switch (c) {
    case 'c': calibrate(); yawRad = 0; prevRate = 0; emit(HEADER); break;
    case 'z': yawRad = 0; prevRate = 0; emitf("#ZERO_YAW,%lu", (unsigned long)millis()); break;
    case 'm': emitf("#MARK,%lu,%lu", (unsigned long)millis(), (unsigned long)++markCount); break;
    case 'H': emit(HEADER); break;
    case 'h': break;                          // only used to learn the WiFi peer
  }
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  delay(300);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_HZ);

#if USE_WIFI
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(200);
  if (WiFi.status() == WL_CONNECTED) { udp.begin(UDP_PORT); emitf("#WIFI,ip=%s,port=%u", WiFi.localIP().toString().c_str(), UDP_PORT); }
  else Serial.println("#WARN,wifi_not_connected_serial_only");
#endif

  if (!mpu.begin(0x68, &Wire)) { emit("#ERR,imu_not_found"); while (1) delay(1000); }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);     // 500 deg/s is plenty for this robot
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);  // keeps fast turns, removes motor buzz

  initTof();
  calibrate();
  emit("#READY");
  emit(HEADER);
  lastImuUs = micros(); lastLogMs = millis();
}

void loop() {
  // ---- commands
  while (Serial.available()) handleCommand((char)Serial.read());
#if USE_WIFI
  int len = udp.parsePacket();
  if (len > 0) {
    char buf[16]; int n = udp.read(buf, sizeof(buf));
    peer = udp.remoteIP(); havePeer = true;
    for (int i = 0; i < n; i++) handleCommand(buf[i]);
  }
#endif

  // ---- IMU at 200 Hz, yaw integrated with the trapezoid rule
  uint32_t nowUs = micros();
  if (nowUs - lastImuUs >= IMU_PERIOD_US) {
    float dt = (nowUs - lastImuUs) * 1e-6f;
    lastImuUs = nowUs;
    sensors_event_t a, g, t; mpu.getEvent(&a, &g, &t);
    float rate = zSign * (g.gyro.z - biasZ);           // counter-clockwise positive, rad/s
    yawRad += 0.5 * (rate + prevRate) * dt;
    prevRate = rate;
    sAx += a.acceleration.x; sAy += a.acceleration.y; sAz += a.acceleration.z;
    sGx += g.gyro.x; sGy += g.gyro.y; sGz += g.gyro.z; nAcc++;
    float ar = fabs(g.gyro.z - biasZ); if (ar > gzPeak) gzPeak = ar;
    pollTof();                                         // cheap: only reads sensors that have a result
  }

  // ---- one log line every LOG_PERIOD_MS
  uint32_t nowMs = millis();
  if (nowMs - lastLogMs >= LOG_PERIOD_MS && nAcc > 0) {
    lastLogMs = nowMs;
    char line[220];
    snprintf(line, sizeof(line),
             "%lu,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.4f,%u,%u,%u,%u",
             (unsigned long)nowMs, tof[0].mm, tof[1].mm, tof[2].mm, tof[3].mm,
             sAx / nAcc, sAy / nAcc, sAz / nAcc, sGx / nAcc, sGy / nAcc, sGz / nAcc,
             yawRad * 57.29578, gzPeak,
             tof[0].status, tof[1].status, tof[2].status, tof[3].status);
    emit(line);
    resetAccumulators();
  }
}
