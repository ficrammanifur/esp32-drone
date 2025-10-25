#include <IBusBM.h>
#include <ESP32Servo.h>  // Include benar untuk ESP32
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define SDA_PIN 21
#define SCL_PIN 22

HardwareSerial IBusSerial(1);
IBusBM ibus;

// ESC outputs (ganti ke Servo class)
Servo esc1; // M1 Front Right (CCW, GPIO14)
Servo esc2; // M2 Front Left  (CW,  GPIO27)
Servo esc3; // M3 Rear Right  (CW,  GPIO26)
Servo esc4; // M4 Rear Left   (CCW, GPIO25)

// Mixer gains (tune sesuai drone: 0.5-1.0; lebih rendah = lebih stabil)
float roll_gain  = 0.7;
float pitch_gain = 0.7;
float yaw_gain   = 0.5;

// MPU6050
MPU6050 accelgyro(0x68); // default address
int16_t ax, ay, az, gx, gy, gz;

void setup() {
  Serial.begin(115200);
  delay(1000);  // Stabilkan serial

  // I2C for MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  accelgyro.initialize();
  // Apply calibrated offsets
  accelgyro.setXAccelOffset(-7493);
  accelgyro.setYAccelOffset(-5781);
  accelgyro.setZAccelOffset(8638);
  accelgyro.setXGyroOffset(-28);
  accelgyro.setYGyroOffset(-6);
  accelgyro.setZGyroOffset(-66);
  Serial.println("MPU6050 initialized with offsets.");

  // iBUS Input (RX GPIO16, baud 115200)
  IBusSerial.begin(115200, SERIAL_8N1, 16, -1);
  ibus.begin(IBusSerial, IBUSBM_NOTIMER);  // Polling manual, hindari konflik timer dengan ESP32Servo

  // ESC PWM output (50Hz default, resolution 16-bit untuk presisi)
  esc1.attach(14, 1000, 2000);  // Pin, min us, max us (kalibrasi otomatis)
  esc2.attach(27, 1000, 2000);
  esc3.attach(26, 1000, 2000);
  esc4.attach(25, 1000, 2000);

  // ARM & Kalibrasi ESC: Mulai 1000 μs, delay 2s (tanpa prop!)
  armESCs();
  Serial.println("Quad Mixer Ready with MPU6050! (NO PROPELLER - Test RC dulu)");
  Serial.println("Throttle <1050 = Failsafe (semua motor OFF)");
}

void loop() {
  ibus.loop();  // Proses iBUS data

  // Read MPU6050 data
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Baca RC channels (CH1-4: Roll, Pitch, Throttle, Yaw)
  int raw_roll  = ibus.readChannel(0);
  int raw_pitch = ibus.readChannel(1);
  int raw_throt = ibus.readChannel(2);
  int raw_yaw   = ibus.readChannel(3);

  // Failsafe: Jika data invalid (-1/0) atau throttle rendah, matikan motor
  if (raw_throt < 1050 || raw_roll < 0 || raw_pitch < 0 || raw_yaw < 0) {
    writeAllESC(1000);  // Semua ke min
    Serial.println("FAILSAVE: Data hilang atau throttle rendah!");
    delay(50);
    return;
  }

  // Normalisasi RC: Center 1500 → 0, range ±500 (asumsi 1000-2000)
  float roll  = (raw_roll  - 1500.0f);  // ±500 us
  float pitch = (raw_pitch - 1500.0f);
  float yaw   = (raw_yaw   - 1500.0f);
  float throttle = raw_throt;  // Tetap 1000-2000 μs

  // X-FRAME MOTOR MIXER (standar Betaflight-style, float presisi)
  float M1 = throttle + (-roll * roll_gain + pitch * pitch_gain - yaw * yaw_gain);  // Front Right: -roll +pitch -yaw
  float M2 = throttle + ( roll * roll_gain + pitch * pitch_gain + yaw * yaw_gain);  // Front Left:  +roll +pitch +yaw
  float M3 = throttle + (-roll * roll_gain - pitch * pitch_gain + yaw * yaw_gain);  // Rear Right:  -roll -pitch +yaw
  float M4 = throttle + ( roll * roll_gain - pitch * pitch_gain - yaw * yaw_gain);  // Rear Left:  +roll -pitch -yaw

  // Limit output ke 1000-2000 μs
  int pwm1 = constrain((int)M1, 1000, 2000);
  int pwm2 = constrain((int)M2, 1000, 2000);
  int pwm3 = constrain((int)M3, 1000, 2000);
  int pwm4 = constrain((int)M4, 1000, 2000);

  // Tulis ke ESC
  esc1.writeMicroseconds(pwm1);
  esc2.writeMicroseconds(pwm2);
  esc3.writeMicroseconds(pwm3);
  esc4.writeMicroseconds(pwm4);

  // Debug: Print RC & Motor & MPU6050 raw data (uncomment #define DEBUG di atas setup() untuk aktifkan)
  #ifdef DEBUG
  Serial.printf("Roll:%d Pitch:%d Throt:%d Yaw:%d | M1:%d M2:%d M3:%d M4:%d | AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d\n",
                raw_roll, raw_pitch, raw_throt, raw_yaw, pwm1, pwm2, pwm3, pwm4, ax, ay, az, gx, gy, gz);
  #endif

  delay(5);  // ~200 Hz loop rate (cepat untuk mixer)
}

// Fungsi helper: Arm semua ESC
void armESCs() {
  writeAllESC(1000);
  delay(2000);  // Biarkan ESC kalibrasi (beep biasanya)
}

// Fungsi helper: Tulis nilai sama ke semua ESC
void writeAllESC(int value) {
  esc1.writeMicroseconds(value);
  esc2.writeMicroseconds(value);
  esc3.writeMicroseconds(value);
  esc4.writeMicroseconds(value);
}
