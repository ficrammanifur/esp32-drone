#include <IBusBM.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>  // Untuk atan2 complementary filter

// iBUS & Serial
HardwareSerial IBusSerial(1);
IBusBM ibus;

// ESC outputs
Servo esc1; // M1 Front Right (CCW, GPIO14)
Servo esc2; // M2 Front Left  (CW,  GPIO27)
Servo esc3; // M3 Rear Right  (CW,  GPIO26)
Servo esc4; // M4 Rear Left   (CCW, GPIO25)

// MPU6050
Adafruit_MPU6050 mpu;

// Offsets dari kalibrasi Anda (hardcode untuk presisi; raw LSB)
float gyro_offset[3]  = {-30.0f, -21.0f, -64.0f};   // Gyro X/Y/Z
float accel_offset[3] = {-9942.0f, -6006.0f, 10957.0f}; // Accel X/Y/Z

// Complementary filter alpha (0.98 = trust gyro lebih)
float alpha = 0.98f;

// PID Controllers (angle outer + rate inner; tune pelan)
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
    dt = (now - last_time) / 1000.0f;  // Seconds
    if (dt < 0.001f) dt = 0.001f;
    last_time = now;

    float error = sp - in;
    integral += error * dt;
    integral = constrain(integral, -50, 50);  // Anti-windup lebih ketat
    float derivative = (error - prev_error) / dt;
    output = kp * error + ki * integral + kd * derivative;
    prev_error = error;
    return output;
  }
};

// PID untuk angle (outer loop: roll/pitch) & rate (inner: yaw/gyro)
PID pid_roll_angle(4.0f, 0.3f, 0.1f);   // Angle roll
PID pid_pitch_angle(4.0f, 0.3f, 0.1f);  // Angle pitch
PID pid_roll_rate(2.5f, 0.1f, 0.05f);   // Rate roll (inner)
PID pid_pitch_rate(2.5f, 0.1f, 0.05f);  // Rate pitch
PID pid_yaw_rate(2.0f, 0.1f, 0.04f);    // Yaw rate

// Armed status (via CH5 >1500)
bool armed = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // I2C Init
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz (cepat setelah fix wiring)

  // iBUS Input
  IBusSerial.begin(115200, SERIAL_8N1, 16, -1);
  ibus.begin(IBusSerial, IBUSBM_NOTIMER);

  // MPU6050 Init
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found! Check wiring.");
    while (1) delay(10);
  }
  Serial.println("MPU6050 found & calibrated!");

  // MPU Config
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Print offsets (dari kalibrasi Anda)
  Serial.printf("Loaded offsets - Gyro: %.0f/%.0f/%.0f | Accel: %.0f/%.0f/%.0f\n",
                gyro_offset[0], gyro_offset[1], gyro_offset[2],
                accel_offset[0], accel_offset[1], accel_offset[2]);

  // ESC Setup
  esc1.attach(14, 1000, 2000);
  esc2.attach(27, 1000, 2000);
  esc3.attach(26, 1000, 2000);
  esc4.attach(25, 1000, 2000);

  // Initial disarm
  writeAllESC(1000);
  delay(2000);
  Serial.println("Quad FC Ready! Arm via CH5 >1500 (NO PROPELLER)");
  Serial.println("Throttle <1050 = Disarm");
}

void loop() {
  unsigned long loop_start = micros();
  unsigned long now = millis();  // FIX: Hitung dt untuk filter
  static unsigned long prev_time = 0;
  float dt = (now - prev_time) / 1000.0f;  // Seconds untuk complementary filter
  if (dt < 0.001f) dt = 0.001f;
  prev_time = now;

  ibus.loop();

  // Baca iBUS
  int raw_roll  = ibus.readChannel(0);  // CH1: Roll setpoint
  int raw_pitch = ibus.readChannel(1);  // CH2: Pitch setpoint
  int raw_throt = ibus.readChannel(2);  // CH3: Throttle
  int raw_yaw   = ibus.readChannel(3);  // CH4: Yaw rate
  int raw_aux1  = ibus.readChannel(4);  // CH5: Arm switch

  // Arm/Disarm
  armed = (raw_aux1 > 1500);
  if (!armed) {
    writeAllESC(1000);
    if (raw_throt > 1050) Serial.println("ARM first! (CH5 up)");
    delay(50);
    return;
  }

  // Failsafe (throttle low)
  if (raw_throt < 1050) {
    writeAllESC(1000);
    Serial.println("FAILSAVE: Throttle low!");
    delay(50);
    return;
  }

  // Scale setpoints (angle ±30° max dari PWM ±500)
  float angle_set_roll  = (raw_roll  - 1500.0f) * 0.06f;  // ±30°
  float angle_set_pitch = (raw_pitch - 1500.0f) * 0.06f;
  float rate_set_yaw    = (raw_yaw   - 1500.0f) * 0.5f;   // ±250°/s

  // Baca MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Raw data dengan offsets (accel ke m/s²; gyro ke °/s)
  float accel_x = (a.acceleration.x - accel_offset[0]) / 16384.0f * 9.81f;
  float accel_y = (a.acceleration.y - accel_offset[1]) / 16384.0f * 9.81f;
  float accel_z = (a.acceleration.z - accel_offset[2]) / 16384.0f * 9.81f;
  float gyro_x  = (g.gyro.x - gyro_offset[0]) * 57.2958f;  // rad/s to °/s
  float gyro_y  = (g.gyro.y - gyro_offset[1]) * 57.2958f;
  float gyro_z  = (g.gyro.z - gyro_offset[2]) * 57.2958f;

  // Complementary filter untuk angle (pitch/roll dari accel + gyro integration) - FIX: Gunakan dt lokal
  static float angle_roll = 0, angle_pitch = 0;
  float accel_roll  = atan2(accel_y, accel_z) * 57.2958f;   // °
  float accel_pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * 57.2958f;
  angle_roll  = alpha * (angle_roll  + gyro_y  * dt) + (1 - alpha) * accel_roll;
  angle_pitch = alpha * (angle_pitch + gyro_x * dt) + (1 - alpha) * accel_pitch;

  // PID: Angle outer → rate target, lalu rate inner
  float rate_target_roll  = pid_roll_angle.compute(angle_set_roll, angle_roll);
  float rate_target_pitch = pid_pitch_angle.compute(angle_set_pitch, angle_pitch);
  pid_roll_rate.compute(rate_target_roll, gyro_y);   // Inner roll rate (gyro Y)
  pid_pitch_rate.compute(rate_target_pitch, -gyro_x); // Inner pitch rate (gyro X, invert)
  float yaw_cmd = pid_yaw_rate.compute(rate_set_yaw, gyro_z);

  // Mixer (throttle + commands; scale PID output ke μs)
  float throttle = raw_throt;
  float roll_cmd  = pid_roll_rate.output * 10.0f;   // Scale ke ±500 μs max
  float pitch_cmd = pid_pitch_rate.output * 10.0f;
  float yaw_cmd_s = yaw_cmd * 8.0f;  // Yaw lebih sensitif

  // X-FRAME
  float M1 = throttle + (-roll_cmd + pitch_cmd - yaw_cmd_s);  // FR
  float M2 = throttle + ( roll_cmd + pitch_cmd + yaw_cmd_s);  // FL
  float M3 = throttle + (-roll_cmd - pitch_cmd + yaw_cmd_s);  // RR
  float M4 = throttle + ( roll_cmd - pitch_cmd - yaw_cmd_s);  // RL

  // Constrain & write
  int pwm1 = constrain((int)M1, 1000, 2000);
  int pwm2 = constrain((int)M2, 1000, 2000);
  int pwm3 = constrain((int)M3, 1000, 2000);
  int pwm4 = constrain((int)M4, 1000, 2000);
  esc1.writeMicroseconds(pwm1);
  esc2.writeMicroseconds(pwm2);
  esc3.writeMicroseconds(pwm3);
  esc4.writeMicroseconds(pwm4);

  // Debug (uncomment #define DEBUG untuk print)
  #ifdef DEBUG
  Serial.printf("Angles:%.1f/%.1f | Rates:%.1f/%.1f/%.1f | PWM:%d/%d/%d/%d | Armed:%s\n",
                angle_roll, angle_pitch, gyro_y, -gyro_x, gyro_z,
                pwm1, pwm2, pwm3, pwm4, armed ? "YES" : "NO");
  #endif

  // Timing 200 Hz
  while (micros() - loop_start < 5000) {}
}

void writeAllESC(int value) {
  esc1.writeMicroseconds(value);
  esc2.writeMicroseconds(value);
  esc3.writeMicroseconds(value);
  esc4.writeMicroseconds(value);
}
