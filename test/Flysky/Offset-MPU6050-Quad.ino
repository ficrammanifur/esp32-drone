#include <IBusBM.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <driver/ledc.h>  // Native ESP32 PWM untuk ESC (lebih stabil & cepat, 400Hz)

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

// Mixer gains (tune sesuai drone: 0.5-1.0; lebih rendah = lebih stabil)
float roll_gain  = 0.7;
float pitch_gain = 0.7;
float yaw_gain   = 0.5;

// MPU6050
MPU6050 accelgyro(0x68); // default address
int16_t ax, ay, az, gx, gy, gz;

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
  ibus.begin(IBusSerial, IBUSBM_NOTIMER);  // Polling manual

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
  delay(3000);

  // Test manual khusus Motor 4 (hapus setelah test)
  Serial.println("Test M4 manual di 1100us...");
  writePWM((ledc_channel_t)PWM_CH4, 1100);
  delay(3000);
  writePWM((ledc_channel_t)PWM_CH4, PWM_MIN);
  Serial.println("Test M4 selesai.");

  Serial.println("Quad Mixer Ready with LEDC PWM (400Hz)! (NO PROPELLER)");
  Serial.println("Throttle <1100 = Failsafe");
}

void loop() {
  ibus.loop();

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  int raw_roll  = ibus.readChannel(0);
  int raw_pitch = ibus.readChannel(1);
  int raw_throt = ibus.readChannel(2);
  int raw_yaw   = ibus.readChannel(3);

  if (raw_throt < 1100 || raw_roll < 0 || raw_pitch < 0 || raw_yaw < 0) {
    writeAllESC(PWM_MIN);
    Serial.println("FAILSAVE!");
    delay(50);
    return;
  }

  float roll  = (raw_roll  - 1500.0f);
  float pitch = (raw_pitch - 1500.0f);
  float yaw   = (raw_yaw   - 1500.0f);
  float throttle = raw_throt;

  float M1 = throttle + (-roll * roll_gain + pitch * pitch_gain - yaw * yaw_gain);
  float M2 = throttle + ( roll * roll_gain + pitch * pitch_gain + yaw * yaw_gain);
  float M3 = throttle + (-roll * roll_gain - pitch * pitch_gain + yaw * yaw_gain);
  float M4 = throttle + ( roll * roll_gain - pitch * pitch_gain - yaw * yaw_gain);

  int pwm1 = constrain((int)M1, PWM_MIN, PWM_MAX);
  int pwm2 = constrain((int)M2, PWM_MIN, PWM_MAX);
  int pwm3 = constrain((int)M3, PWM_MIN, PWM_MAX);
  int pwm4 = constrain((int)M4, PWM_MIN, PWM_MAX);

  // Tulis M4 dulu
  writePWM((ledc_channel_t)PWM_CH4, pwm4);
  writePWM((ledc_channel_t)PWM_CH1, pwm1);
  writePWM((ledc_channel_t)PWM_CH2, pwm2);
  writePWM((ledc_channel_t)PWM_CH3, pwm3);

  #ifdef DEBUG
  Serial.printf("Roll:%d Pitch:%d Throt:%d Yaw:%d | M1:%d M2:%d M3:%d M4:%d\n",
                raw_roll, raw_pitch, raw_throt, raw_yaw, pwm1, pwm2, pwm3, pwm4);
  #endif

  delay(2);
}
