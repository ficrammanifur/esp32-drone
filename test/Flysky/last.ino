#include <IBusBM.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <driver/ledc.h>  // Native ESP32 PWM untuk ESC (lebih stabil & cepat, 400Hz)
#include <math.h>  // Untuk atan2, sqrt (M_PI tidak diperlukan karena Arduino.h punya DEG_TO_RAD/RAD_TO_DEG)

#define SDA_PIN 21
#define SCL_PIN 22

HardwareSerial IBusSerial(1);
IBusBM ibus;

// Define LEDC PWM channels untuk ESC (0-15 available, gunakan timer 0)
#define PWM_CH1 0  // Motor 1: Front Right (CCW, GPIO14)
#define PWM_CH2 1  // Motor 2: Front Left  (CW,  GPIO27)
#define PWM_CH3 2  // Motor 3: Rear Right  (CW,  GPIO26)
#define PWM_CH4 3  // Motor 4: Rear Left   (CCW, GPIO25)
#define PWM_FREQ 400  // Hz (naikkin dari 50Hz default Servo → respons lebih cepat, kurang jitter)
#define PWM_RES LEDC_TIMER_13_BIT    // 13-bit resolution (8191 max, cukup buat 1000-2000us presisi)
#define PWM_MIN 1000  // us minimum
#define PWM_MAX 2000  // us maximum

// Angle Mode Constants
const float MAX_ANGLE = 30.0f;  // Max desired angle deg (RC stick full = ±30°)
const float RC_RANGE = 500.0f;  // 1500 ±500 us for full range
const float GRAVITY = 9.81f;    // Untuk scaling accel jika perlu

// PID coefficients untuk Angle Mode (tune awal: P-only, lalu +D, +I)
float Kp_roll = 2.0f, Ki_roll = 0.0f, Kd_roll = 0.1f;  // Mulai dengan P=2.0, D=0.1
float Kp_pitch = 2.0f, Ki_pitch = 0.0f, Kd_pitch = 0.1f;
float Kp_yaw = 1.0f, Ki_yaw = 0.0f, Kd_yaw = 0.05f;    // Yaw lebih rendah

// PID state untuk Angle
float error_roll_prev = 0, integral_roll = 0;
float error_pitch_prev = 0, integral_pitch = 0;
float error_yaw_prev = 0, integral_yaw = 0;

// Yaw rate PID (inner loop untuk yaw, karena yaw sulit dari accel)
float Kp_yaw_rate = 0.5f, Ki_yaw_rate = 0.0f, Kd_yaw_rate = 0.0f;
float error_yaw_rate_prev = 0, integral_yaw_rate = 0;

// PID tuning constants
const float GYRO_SENS = 131.0f;  // LSB per deg/s for +/-250 deg/s range (default MPU6050)
const float RC_CENTER = 1500.0f;
const float RC_DEADZONE = 20.0f;  // Deadzone for RC inputs

// Timing for PID
unsigned long prev_micros = 0;

// Throttle arm threshold
int throttle_arm_min = 1050;

// MPU6050
MPU6050 accelgyro(0x68); // default address
int16_t ax, ay, az, gx, gy, gz;

// Variabel untuk auto kalibrasi
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
const int CALIB_SAMPLES = 200;  // Jumlah sampel untuk rata-rata kalibrasi

// Fungsi helper: Konversi us ke duty cycle LEDC
uint32_t usToDuty(int us) {
  uint32_t period_us = 1000000 / PWM_FREQ;
  return ((uint64_t)(us) * (1ULL << PWM_RES)) / period_us;
}

// Fungsi helper: Tulis PWM ke channel
void writePWM(ledc_channel_t channel, int us) {
  uint32_t duty = usToDuty(constrain(us, PWM_MIN, PWM_MAX));
  ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

// Fungsi helper: Tulis nilai sama ke semua ESC
void writeAllESC(int value) {
  writePWM((ledc_channel_t)PWM_CH4, value);  // Prioritas M4
  writePWM((ledc_channel_t)PWM_CH1, value);
  writePWM((ledc_channel_t)PWM_CH2, value);
  writePWM((ledc_channel_t)PWM_CH3, value);
}

// Fungsi: Auto kalibrasi MPU6050 (drone harus datar & diam saat setup)
void autoCalibrateMPU() {
  Serial.println("=== AUTO KALIBRASI MPU6050 ===");
  Serial.println("Taruh drone DATAR & DIAM selama 5 detik. Jangan gerakkan!");
  
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  
  delay(2000);  // Waktu tunggu
  
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_sum += ax; ay_sum += ay; az_sum += az;
    gx_sum += gx; gy_sum += gy; gz_sum += gz;
    
    if (i % 50 == 0) {
      Serial.printf("Kalibrasi sampel %d/%d...\n", i, CALIB_SAMPLES);
    }
    
    delay(25);  // ~40Hz sampling
  }
  
  // Hitung offsets (rata-rata)
  ax_offset = ax_sum / CALIB_SAMPLES;
  ay_offset = ay_sum / CALIB_SAMPLES;
  az_offset = (az_sum / CALIB_SAMPLES) - 16384;  // Az harus ~1g (16384 LSB), kurangi bias
  gx_offset = gx_sum / CALIB_SAMPLES;
  gy_offset = gy_sum / CALIB_SAMPLES;
  gz_offset = gz_sum / CALIB_SAMPLES;
  
  // Apply offsets ke MPU
  accelgyro.setXAccelOffset(ax_offset);
  accelgyro.setYAccelOffset(ay_offset);
  accelgyro.setZAccelOffset(az_offset);
  accelgyro.setXGyroOffset(gx_offset);
  accelgyro.setYGyroOffset(gy_offset);
  accelgyro.setZGyroOffset(gz_offset);
  
  Serial.printf("Offsets diterapkan:\n");
  Serial.printf("Accel: X=%d, Y=%d, Z=%d\n", ax_offset, ay_offset, az_offset);
  Serial.printf("Gyro:  X=%d, Y=%d, Z=%d\n", gx_offset, gy_offset, gz_offset);
  Serial.println("Kalibrasi selesai! Siap untuk PID Angle Mode.");
}

// Fungsi: Hitung sudut roll & pitch dari accelerometer (angle mode)
void computeAngles(float &roll_angle, float &pitch_angle) {
  // Raw accel values (sudah offset-corrected)
  float accel_x = ax / 16384.0f;  // +/-2g default scale
  float accel_y = ay / 16384.0f;
  float accel_z = az / 16384.0f;

  // Roll (X-axis tilt)
  roll_angle = atan2(accel_y, accel_z) * RAD_TO_DEG;

  // Pitch (Y-axis tilt)
  float accel_yz = sqrt(accel_y * accel_y + accel_z * accel_z);
  pitch_angle = atan2(-accel_x, accel_yz) * RAD_TO_DEG;
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Stabilkan serial

  // I2C for MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  accelgyro.initialize();
  Serial.println("MPU6050 initialized (sebelum kalibrasi).");

  // iBUS Input (RX GPIO16, baud 115200)
  IBusSerial.begin(115200, SERIAL_8N1, 16, -1);
  ibus.begin(IBusSerial, IBUSBM_NOTIMER);  // Polling manual
  delay(2000);  // Tunggu iBUS connect
  Serial.println("iBUS started. Tunggu signal dari receiver...");

  // AUTO KALIBRASI: Jalankan di sini (ganti hardcoded offsets)
  autoCalibrateMPU();

  // Setup LEDC Timer (LOW_SPEED_MODE, 400Hz, 13-bit res)
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = PWM_RES,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  // Channel 1: Pin 14
  ledc_channel_config_t ledc_ch1 = {
    .gpio_num = 14,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = (ledc_channel_t)PWM_CH1,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
    .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    .flags = {
      .output_invert = 0
    }
  };
  ledc_channel_config(&ledc_ch1);

  // Channel 2: Pin 27
  ledc_channel_config_t ledc_ch2 = {
    .gpio_num = 27,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = (ledc_channel_t)PWM_CH2,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
    .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    .flags = {
      .output_invert = 0
    }
  };
  ledc_channel_config(&ledc_ch2);

  // Channel 3: Pin 26
  ledc_channel_config_t ledc_ch3 = {
    .gpio_num = 26,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = (ledc_channel_t)PWM_CH3,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
    .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    .flags = {
      .output_invert = 0
    }
  };
  ledc_channel_config(&ledc_ch3);

  // Channel 4: Pin 25
  ledc_channel_config_t ledc_ch4 = {
    .gpio_num = 25,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = (ledc_channel_t)PWM_CH4,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0,
    .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    .flags = {
      .output_invert = 0
    }
  };
  ledc_channel_config(&ledc_ch4);

  // ARM & Kalibrasi ESC: Mulai 1000 μs, delay 3s
  writeAllESC(PWM_MIN);
  Serial.println("ESC armed di 1000us. Delay 3s...");
  delay(3000);

  // Test manual khusus Motor 4
  Serial.println("Test M4 manual di 1100us...");
  writePWM((ledc_channel_t)PWM_CH4, 1100);
  delay(3000);
  writePWM((ledc_channel_t)PWM_CH4, PWM_MIN);
  Serial.println("Test M4 selesai.");

  prev_micros = micros();  // Init timing

  Serial.println("\n=== QUAD READY: ANGLE MODE (Hover Stabil) ===");
  Serial.println("Motor Mapping:");
  Serial.println("M1 (FR, Pin14): " + String(PWM_CH1));
  Serial.println("M2 (FL, Pin27): " + String(PWM_CH2));
  Serial.println("M3 (RR, Pin26): " + String(PWM_CH3));
  Serial.println("M4 (RL, Pin25): " + String(PWM_CH4));
  Serial.println("Throttle arm: " + String(throttle_arm_min) + " us");
  Serial.println("Monitor: Lihat 'MOTOR PWM:' untuk nilai terpisah M1-M4");
}

void loop() {
  unsigned long current_micros = micros();
  float dt = (current_micros - prev_micros) / 1000000.0f;
  prev_micros = current_micros;
  if (dt > 0.01f) dt = 0.002f;  // Clamp dt untuk stabilitas

  ibus.loop();

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  int raw_roll  = ibus.readChannel(0);  // RC stick roll → desired roll angle
  int raw_pitch = ibus.readChannel(1);  // RC stick pitch → desired pitch angle
  int raw_throt = ibus.readChannel(2);  // Throttle
  int raw_yaw   = ibus.readChannel(3);  // RC stick yaw → desired yaw rate (rate mode untuk yaw)

  // DEBUG RAW channels (lebih jarang)
  static int debug_counter = 0;
  debug_counter++;
  if (debug_counter % 100 == 0) {
    Serial.printf("RAW: R=%d P=%d T=%d Y=%d\n", raw_roll, raw_pitch, raw_throt, raw_yaw);
  }

  if (raw_throt < throttle_arm_min || raw_roll < 0 || raw_pitch < 0 || raw_yaw < 0) {
    writeAllESC(PWM_MIN);
    // Reset PID states
    integral_roll = integral_pitch = integral_yaw = integral_yaw_rate = 0;
    error_roll_prev = error_pitch_prev = error_yaw_prev = error_yaw_rate_prev = 0;
    Serial.println("FAILSAVE!");
    delay(50);
    return;
  }

  // Apply deadzone to RC
  if (abs(raw_roll - RC_CENTER) < RC_DEADZONE) raw_roll = (int)RC_CENTER;
  if (abs(raw_pitch - RC_CENTER) < RC_DEADZONE) raw_pitch = (int)RC_CENTER;
  if (abs(raw_yaw - RC_CENTER) < RC_DEADZONE) raw_yaw = (int)RC_CENTER;

  // Compute current angles from accel (measured_roll, measured_pitch)
  float measured_roll, measured_pitch;
  computeAngles(measured_roll, measured_pitch);

  // Setpoints: RC to desired angle (deg) untuk roll/pitch, rate untuk yaw
  float setpoint_roll = (raw_roll - RC_CENTER) / RC_RANGE * MAX_ANGLE;
  float setpoint_pitch = (raw_pitch - RC_CENTER) / RC_RANGE * MAX_ANGLE;
  float setpoint_yaw_rate = (raw_yaw - RC_CENTER) / RC_RANGE * 180.0f;  // Yaw: ±180 deg/s max

  // Gyro rates (deg/s) untuk yaw derivative
  float gyro_yaw = gz / GYRO_SENS;

  // === PID ROLL (Angle Mode) ===
  float error_roll = setpoint_roll - measured_roll;
  integral_roll += error_roll * dt;
  integral_roll = constrain(integral_roll, -50.0f, 50.0f);  // Anti-windup (sudut terbatas)
  float derivative_roll = (error_roll - error_roll_prev) / dt;
  float pid_roll = Kp_roll * error_roll + Ki_roll * integral_roll + Kd_roll * derivative_roll;
  error_roll_prev = error_roll;

  // === PID PITCH (Angle Mode) ===
  float error_pitch = setpoint_pitch - measured_pitch;
  integral_pitch += error_pitch * dt;
  integral_pitch = constrain(integral_pitch, -50.0f, 50.0f);
  float derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  float pid_pitch = Kp_pitch * error_pitch + Ki_pitch * integral_pitch + Kd_pitch * derivative_pitch;
  error_pitch_prev = error_pitch;

  // === PID YAW (Rate Mode, inner loop) ===
  float error_yaw_rate = setpoint_yaw_rate - gyro_yaw;
  integral_yaw_rate += error_yaw_rate * dt;
  integral_yaw_rate = constrain(integral_yaw_rate, -100.0f, 100.0f);
  float derivative_yaw_rate = (error_yaw_rate - error_yaw_rate_prev) / dt;
  float pid_yaw = Kp_yaw_rate * error_yaw_rate + Ki_yaw_rate * integral_yaw_rate + Kd_yaw_rate * derivative_yaw_rate;
  error_yaw_rate_prev = error_yaw_rate;

  float throttle = raw_throt;

  // Mixer: Gunakan PID output langsung sebagai correction (dalam us, tuned via Kp ~ us per deg error)
  // Hover: Saat RC center (setpoint=0), PID counter error untuk jaga angle=0
  float M1 = throttle - pid_roll + pid_pitch - pid_yaw;  // Front Right
  float M2 = throttle + pid_roll + pid_pitch + pid_yaw;  // Front Left
  float M3 = throttle - pid_roll - pid_pitch + pid_yaw;  // Rear Right
  float M4 = throttle + pid_roll - pid_pitch - pid_yaw;  // Rear Left

  int pwm1 = constrain((int)M1, PWM_MIN, PWM_MAX);
  int pwm2 = constrain((int)M2, PWM_MIN, PWM_MAX);
  int pwm3 = constrain((int)M3, PWM_MIN, PWM_MAX);
  int pwm4 = constrain((int)M4, PWM_MIN, PWM_MAX);

  // Tulis PWM (M4 dulu untuk safety)
  writePWM((ledc_channel_t)PWM_CH4, pwm4);
  writePWM((ledc_channel_t)PWM_CH1, pwm1);
  writePWM((ledc_channel_t)PWM_CH2, pwm2);
  writePWM((ledc_channel_t)PWM_CH3, pwm3);

  // DEBUG: Print angles & PID setiap 100 loops
  if (debug_counter % 100 == 0) {
    Serial.printf("Angles: SetR=%.1f MeasR=%.1f PID_R=%.1f | SetP=%.1f MeasP=%.1f PID_P=%.1f | PID_Y=%.1f\n",
                  setpoint_roll, measured_roll, pid_roll,
                  setpoint_pitch, measured_pitch, pid_pitch,
                  pid_yaw);
  }

  // MOTOR PWM TERPISAH: Print khusus untuk M1-M4 (setiap 50 loops, lebih jelas)
  if (debug_counter % 50 == 0) {
    Serial.printf("MOTOR PWM: M1(FR)=%d | M2(FL)=%d | M3(RR)=%d | M4(RL)=%d | Throt=%d\n",
                  pwm1, pwm2, pwm3, pwm4, (int)throttle);
  }

  delay(2);  // ~500Hz loop rate
}
