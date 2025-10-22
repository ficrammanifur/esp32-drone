#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <HardwareSerial.h>
#include <TinyGPS++.h>

// MPU6050 Object
MPU6050 mpu(0x68);

// Offsets hasil kalibrasi
int16_t ax_offset = -7477;
int16_t ay_offset = -5665;
int16_t az_offset = 8937;
int16_t gx_offset = -28;
int16_t gy_offset = -21;
int16_t gz_offset = -71;

// Variabel sensor
int16_t ax, ay, az;
int16_t gx, gy, gz;

float roll = 0, pitch = 0, yaw = 0;
float roll0 = 0, pitch0 = 0, yaw0 = 0; // offset awal
float dt = 0.01; 
float alpha = 0.98; 

unsigned long timer;

// GPS Setup
HardwareSerial GPS(1);
TinyGPSPlus gps;
#define GPS_RX 16
#define GPS_TX 17

void setup() {
  Wire.begin(); 
  Serial.begin(115200);

  // MPU6050 init
  mpu.initialize();
  Serial.println("MPU6050 initialized.");
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);

  // GPS init
  GPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // =================== Kalibrasi awal ===================
  Serial.println("Calibrating... Keep the sensor flat!");
  int samples = 100;
  float r=0, p=0, y=0;
  for(int i=0; i<samples; i++){
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float gyroXrate = gx / 131.0;
    float gyroYrate = gy / 131.0;
    float gyroZrate = gz / 131.0;

    float accX = ax / 16384.0;
    float accY = ay / 16384.0;
    float accZ = az / 16384.0;

    float rollAcc  = atan2(accY, accZ) * 57.2958;
    float pitchAcc = atan(-accX / sqrt(accY*accY + accZ*accZ)) * 57.2958;

    r += rollAcc;
    p += pitchAcc;
    delay(10);
  }
  roll0 = r / samples;
  pitch0 = p / samples;
  yaw0 = 0; // yaw tanpa magnetometer, mulai dari 0
  timer = micros();
  Serial.println("Calibration done.");
}

void loop() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float gyroXrate = gx / 131.0;
  float gyroYrate = gy / 131.0;
  float gyroZrate = gz / 131.0;

  float accX = ax / 16384.0;
  float accY = ay / 16384.0;
  float accZ = az / 16384.0;

  float rollAcc  = atan2(accY, accZ) * 57.2958;
  float pitchAcc = atan(-accX / sqrt(accY*accY + accZ*accZ)) * 57.2958;

  unsigned long now = micros();
  dt = (now - timer) / 1000000.0;
  timer = now;

  roll  = alpha * (roll + gyroXrate * dt)  + (1 - alpha) * rollAcc;
  pitch = alpha * (pitch + gyroYrate * dt) + (1 - alpha) * pitchAcc;
  yaw   += gyroZrate * dt;

  // Subtract initial offsets
  float rollOut  = roll - roll0;
  float pitchOut = pitch - pitch0;
  float yawOut   = yaw - yaw0;

  Serial.print("Roll: ");  Serial.print(rollOut, 2);
  Serial.print("\tPitch: "); Serial.print(pitchOut, 2);
  Serial.print("\tYaw: ");   Serial.println(yawOut, 2);

  // ================= GPS =================
  while (GPS.available() > 0) {
    char c = GPS.read();
    gps.encode(c);
  }

  if (gps.location.isUpdated()) {
    Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
    Serial.print("\tLng: "); Serial.print(gps.location.lng(), 6);
    Serial.print("\tAlt: "); Serial.println(gps.altitude.meters());
  }

  delay(10);
}
