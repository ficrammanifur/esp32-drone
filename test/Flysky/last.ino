#include <IBusBM.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <math.h>

// WiFi AP Config
const char* ap_ssid = "FRISS";
const char* ap_password = "mamahfris";
WebServer server(80);

// iBUS & Serial
HardwareSerial IBusSerial(1);
HardwareSerial GPSSerial(2);
IBusBM ibus;
TinyGPSPlus gps;

// ESC outputs
Servo esc1, esc2, esc3, esc4;

// MPU6050
Adafruit_MPU6050 mpu;

// GPS & LoRa Config (dari EEPROM)
struct Config {
  bool lora_enabled;
  bool gps_enabled;
  uint32_t checksum;
} config;

// Telemetry data
struct TelemetryData {
  float lat;
  float lon;
  float yaw;
  int throttle;
  float battery;
  unsigned long timestamp;
} telemetry;

// Offsets kalibrasi
float gyro_offset[3]  = {-30.0f, -21.0f, -64.0f};
float accel_offset[3] = {-9942.0f, -6006.0f, 10957.0f};
float alpha = 0.98f;

// PID Controllers
class PID {
public:
  float kp, ki, kd;
  float setpoint, input, output;
  float integral, prev_error;
  unsigned long last_time;
  float dt;

  PID(float p, float i, float d) : kp(p), ki(i), kd(d), integral(0), prev_error(0), last_time(0) {}

  float compute(float sp, float in) {
    unsigned long now = millis();
    dt = (now - last_time) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    last_time = now;
    float error = sp - in;
    integral += error * dt;
    integral = constrain(integral, -50, 50);
    float derivative = (error - prev_error) / dt;
    output = kp * error + ki * integral + kd * derivative;
    prev_error = error;
    return output;
  }
};

PID pid_roll_angle(4.0f, 0.3f, 0.1f);
PID pid_pitch_angle(4.0f, 0.3f, 0.1f);
PID pid_roll_rate(2.5f, 0.1f, 0.05f);
PID pid_pitch_rate(2.5f, 0.1f, 0.05f);
PID pid_yaw_rate(2.0f, 0.1f, 0.04f);

bool armed = false;
float angle_roll = 0, angle_pitch = 0;

// ============ EEPROM Functions ============
void loadConfig() {
  EEPROM.begin(512);
  EEPROM.get(0, config);
  
  uint32_t calc_checksum = config.lora_enabled + config.gps_enabled;
  if (config.checksum != calc_checksum) {
    // Default config
    config.lora_enabled = false;
    config.gps_enabled = false;
    saveConfig();
  }
  Serial.printf("Config loaded: LoRa=%d, GPS=%d\n", config.lora_enabled, config.gps_enabled);
}

void saveConfig() {
  config.checksum = config.lora_enabled + config.gps_enabled;
  EEPROM.put(0, config);
  EEPROM.commit();
  Serial.println("Config saved to EEPROM");
}

// ============ Web Server Handlers ============
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>FRISS Ground Station</title>
  <style>
    body { font-family: Arial; margin: 0; padding: 20px; background: #1a1a2e; color: #eee; }
    .container { max-width: 800px; margin: 0 auto; }
    h1 { text-align: center; color: #0f3; }
    .card { background: #16213e; padding: 20px; margin: 15px 0; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
    .label { color: #aaa; font-size: 14px; margin-bottom: 5px; }
    .value { font-size: 24px; color: #0f3; font-weight: bold; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
    button { background: #0f3; color: #000; border: none; padding: 15px 30px; font-size: 16px; border-radius: 5px; cursor: pointer; width: 100%; margin-top: 10px; }
    button:hover { background: #0c0; }
    .switch { position: relative; display: inline-block; width: 60px; height: 34px; }
    .switch input { opacity: 0; width: 0; height: 0; }
    .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; transition: .4s; border-radius: 34px; }
    .slider:before { position: absolute; content: ""; height: 26px; width: 26px; left: 4px; bottom: 4px; background-color: white; transition: .4s; border-radius: 50%; }
    input:checked + .slider { background-color: #0f3; }
    input:checked + .slider:before { transform: translateX(26px); }
  </style>
</head>
<body>
  <div class="container">
    <h1>🚁 FRISS Ground Station</h1>
    
    <div class="card">
      <h2>📡 Telemetry Data</h2>
      <div class="grid">
        <div>
          <div class="label">Latitude</div>
          <div class="value" id="lat">--</div>
        </div>
        <div>
          <div class="label">Longitude</div>
          <div class="value" id="lon">--</div>
        </div>
        <div>
          <div class="label">Yaw (°)</div>
          <div class="value" id="yaw">--</div>
        </div>
        <div>
          <div class="label">Throttle</div>
          <div class="value" id="throttle">--</div>
        </div>
        <div>
          <div class="label">Battery (V)</div>
          <div class="value" id="battery">--</div>
        </div>
        <div>
          <div class="label">GPS Satellites</div>
          <div class="value" id="sats">--</div>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>⚙️ Configuration</h2>
      <div style="margin: 15px 0; display: flex; justify-content: space-between; align-items: center;">
        <span>Enable LoRa Telemetry</span>
        <label class="switch">
          <input type="checkbox" id="lora" onchange="updateConfig()">
          <span class="slider"></span>
        </label>
      </div>
      <div style="margin: 15px 0; display: flex; justify-content: space-between; align-items: center;">
        <span>Enable GPS Monitoring</span>
        <label class="switch">
          <input type="checkbox" id="gps" onchange="updateConfig()">
          <span class="slider"></span>
        </label>
      </div>
      <button onclick="saveConfig()">💾 SAVE CONFIG</button>
    </div>
  </div>

  <script>
    function updateTelemetry() {
      fetch('/telemetry')
        .then(r => r.json())
        .then(data => {
          document.getElementById('lat').textContent = data.lat.toFixed(6);
          document.getElementById('lon').textContent = data.lon.toFixed(6);
          document.getElementById('yaw').textContent = data.yaw.toFixed(1);
          document.getElementById('throttle').textContent = data.throttle;
          document.getElementById('battery').textContent = data.battery.toFixed(2);
          document.getElementById('sats').textContent = data.sats;
        });
    }

    function loadConfig() {
      fetch('/config')
        .then(r => r.json())
        .then(data => {
          document.getElementById('lora').checked = data.lora;
          document.getElementById('gps').checked = data.gps;
        });
    }

    function updateConfig() {
      // Visual feedback only, actual save on button click
    }

    function saveConfig() {
      const data = {
        lora: document.getElementById('lora').checked,
        gps: document.getElementById('gps').checked
      };
      fetch('/config', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(data)
      })
      .then(r => r.text())
      .then(msg => {
        alert(msg);
      });
    }

    loadConfig();
    setInterval(updateTelemetry, 1000);
    updateTelemetry();
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleTelemetry() {
  String json = "{";
  json += "\"lat\":" + String(telemetry.lat, 6) + ",";
  json += "\"lon\":" + String(telemetry.lon, 6) + ",";
  json += "\"yaw\":" + String(telemetry.yaw, 1) + ",";
  json += "\"throttle\":" + String(telemetry.throttle) + ",";
  json += "\"battery\":" + String(telemetry.battery, 2) + ",";
  json += "\"sats\":" + String(gps.satellites.value());
  json += "}";
  server.send(200, "application/json", json);
}

void handleGetConfig() {
  String json = "{";
  json += "\"lora\":" + String(config.lora_enabled ? "true" : "false") + ",";
  json += "\"gps\":" + String(config.gps_enabled ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handlePostConfig() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    
    // Simple JSON parsing
    config.lora_enabled = body.indexOf("\"lora\":true") > 0;
    config.gps_enabled = body.indexOf("\"gps\":true") > 0;
    
    saveConfig();
    
    // Reinit modules
    if (config.lora_enabled) initLoRa();
    if (config.gps_enabled) initGPS();
    
    server.send(200, "text/plain", "Config saved! LoRa=" + String(config.lora_enabled) + " GPS=" + String(config.gps_enabled));
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

// ============ LoRa Functions ============
void initLoRa() {
  SPI.begin(18, 19, 23, 5); // SCK, MISO, MOSI, NSS
  LoRa.setPins(5, 4, 2);    // NSS, RST, DIO0
  
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    config.lora_enabled = false;
    return;
  }
  
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  Serial.println("LoRa initialized at 433 MHz");
}

void sendLoRaTelemetry() {
  if (!config.lora_enabled) return;
  
  LoRa.beginPacket();
  LoRa.print("LAT:");  LoRa.print(telemetry.lat, 6);
  LoRa.print(",LON:"); LoRa.print(telemetry.lon, 6);
  LoRa.print(",YAW:"); LoRa.print(telemetry.yaw, 1);
  LoRa.print(",THR:"); LoRa.print(telemetry.throttle);
  LoRa.print(",BAT:"); LoRa.print(telemetry.battery, 2);
  LoRa.endPacket();
}

// ============ GPS Functions ============
void initGPS() {
  GPSSerial.begin(9600, SERIAL_8N1, 17, -1); // RX=17, TX not used
  Serial.println("GPS initialized on GPIO17");
}

void updateGPS() {
  if (!config.gps_enabled) return;
  
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }
  
  if (gps.location.isUpdated()) {
    telemetry.lat = gps.location.lat();
    telemetry.lon = gps.location.lng();
  }
}

// ============ Battery Monitor ============
float readBattery() {
  // Asumsi battery monitor di GPIO 34 (ADC1_CH6), voltage divider 11:1
  int raw = analogRead(34);
  return (raw / 4095.0) * 3.3 * 11.0; // Scale to voltage
}

// ============ Main Setup ============
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Load config from EEPROM
  loadConfig();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.println("WiFi AP Started: " + String(ap_ssid));
  Serial.println("IP: " + WiFi.softAPIP().toString());

  // Web Server Routes
  server.on("/", handleRoot);
  server.on("/telemetry", handleTelemetry);
  server.on("/config", HTTP_GET, handleGetConfig);
  server.on("/config", HTTP_POST, handlePostConfig);
  server.begin();
  Serial.println("Web Server started on port 80");

  // I2C & MPU6050
  Wire.begin();
  Wire.setClock(400000);
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 initialized");

  // iBUS
  IBusSerial.begin(115200, SERIAL_8N1, 16, -1);
  ibus.begin(IBusSerial, IBUSBM_NOTIMER);

  // ESCs
  esc1.attach(14, 1000, 2000);
  esc2.attach(27, 1000, 2000);
  esc3.attach(26, 1000, 2000);
  esc4.attach(25, 1000, 2000);
  writeAllESC(1000);
  delay(2000);

  // Optional modules
  if (config.lora_enabled) initLoRa();
  if (config.gps_enabled) initGPS();

  Serial.println("Quad FC Ready! Connect to WiFi: FRISS / mamahfris");
  Serial.println("Open browser: http://192.168.4.1");
}

// ============ Main Loop ============
void loop() {
  unsigned long loop_start = micros();
  unsigned long now = millis();
  static unsigned long prev_time = 0;
  static unsigned long prev_telemetry = 0;
  
  float dt = (now - prev_time) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  prev_time = now;

  // Handle web clients
  server.handleClient();

  // Update GPS
  updateGPS();

  // iBUS
  ibus.loop();
  int raw_roll  = ibus.readChannel(0);
  int raw_pitch = ibus.readChannel(1);
  int raw_throt = ibus.readChannel(2);
  int raw_yaw   = ibus.readChannel(3);
  int raw_aux1  = ibus.readChannel(4);

  // Arm/Disarm
  armed = (raw_aux1 > 1500);
  if (!armed || raw_throt < 1050) {
    writeAllESC(1000);
    delay(50);
    return;
  }

  // Scale setpoints
  float angle_set_roll  = (raw_roll  - 1500.0f) * 0.06f;
  float angle_set_pitch = (raw_pitch - 1500.0f) * 0.06f;
  float rate_set_yaw    = (raw_yaw   - 1500.0f) * 0.5f;

  // Read MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accel_x = (a.acceleration.x - accel_offset[0]) / 16384.0f * 9.81f;
  float accel_y = (a.acceleration.y - accel_offset[1]) / 16384.0f * 9.81f;
  float accel_z = (a.acceleration.z - accel_offset[2]) / 16384.0f * 9.81f;
  float gyro_x  = (g.gyro.x - gyro_offset[0]) * 57.2958f;
  float gyro_y  = (g.gyro.y - gyro_offset[1]) * 57.2958f;
  float gyro_z  = (g.gyro.z - gyro_offset[2]) * 57.2958f;

  // Complementary filter
  float accel_roll  = atan2(accel_y, accel_z) * 57.2958f;
  float accel_pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * 57.2958f;
  angle_roll  = alpha * (angle_roll  + gyro_y * dt) + (1 - alpha) * accel_roll;
  angle_pitch = alpha * (angle_pitch + gyro_x * dt) + (1 - alpha) * accel_pitch;

  // PID control
  float rate_target_roll  = pid_roll_angle.compute(angle_set_roll, angle_roll);
  float rate_target_pitch = pid_pitch_angle.compute(angle_set_pitch, angle_pitch);
  pid_roll_rate.compute(rate_target_roll, gyro_y);
  pid_pitch_rate.compute(rate_target_pitch, -gyro_x);
  float yaw_cmd = pid_yaw_rate.compute(rate_set_yaw, gyro_z);

  // Mixer
  float throttle = raw_throt;
  float roll_cmd  = pid_roll_rate.output * 10.0f;
  float pitch_cmd = pid_pitch_rate.output * 10.0f;
  float yaw_cmd_s = yaw_cmd * 8.0f;

  float M1 = throttle + (-roll_cmd + pitch_cmd - yaw_cmd_s);
  float M2 = throttle + ( roll_cmd + pitch_cmd + yaw_cmd_s);
  float M3 = throttle + (-roll_cmd - pitch_cmd + yaw_cmd_s);
  float M4 = throttle + ( roll_cmd - pitch_cmd - yaw_cmd_s);

  int pwm1 = constrain((int)M1, 1000, 2000);
  int pwm2 = constrain((int)M2, 1000, 2000);
  int pwm3 = constrain((int)M3, 1000, 2000);
  int pwm4 = constrain((int)M4, 1000, 2000);
  
  esc1.writeMicroseconds(pwm1);
  esc2.writeMicroseconds(pwm2);
  esc3.writeMicroseconds(pwm3);
  esc4.writeMicroseconds(pwm4);

  // Update telemetry (every 500ms)
  if (now - prev_telemetry > 500) {
    telemetry.yaw = gyro_z;
    telemetry.throttle = raw_throt;
    telemetry.battery = readBattery();
    telemetry.timestamp = now;
    
    sendLoRaTelemetry();
    prev_telemetry = now;
  }

  // Maintain 200Hz loop
  while (micros() - loop_start < 5000) {}
}

void writeAllESC(int value) {
  esc1.writeMicroseconds(value);
  esc2.writeMicroseconds(value);
  esc3.writeMicroseconds(value);
  esc4.writeMicroseconds(value);
}
