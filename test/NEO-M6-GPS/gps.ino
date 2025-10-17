#include <Wire.h>
#include "MPU6050.h"
#include <TinyGPS++.h>

#define SDA_PIN 21
#define SCL_PIN 22

MPU6050 accelgyro;

// Offsets hasil kalibrasi
int ax_offset = -7844;
int ay_offset = -5615;
int az_offset = 8636;
int gx_offset = -26;
int gy_offset = -8;
int gz_offset = -67;

int16_t ax, ay, az, gx, gy, gz;

// GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // UART1

#define GPS_RX 16  // RX pin ESP32 ke TX Neo-6M
#define GPS_TX 17  // TX pin ESP32 ke RX Neo-6M (optional, kadang tidak perlu)

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Initializing MPU6050...");
  accelgyro.initialize();

  // Terapkan offsets kalibrasi
  accelgyro.setXAccelOffset(ax_offset);
  accelgyro.setYAccelOffset(ay_offset);
  accelgyro.setZAccelOffset(az_offset);
  accelgyro.setXGyroOffset(gx_offset);
  accelgyro.setYGyroOffset(gy_offset);
  accelgyro.setZGyroOffset(gz_offset);

  if(accelgyro.testConnection()){
    Serial.println("MPU6050 connected successfully!");
  } else {
    Serial.println("MPU6050 connection failed!");
    while(1);
  }

  // Setup GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); 
  Serial.println("GPS initialized...");
}

void loop() {
  // ----- MPU6050 -----
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("Accel [X Y Z]: ");
  Serial.print(ax); Serial.print(" ");
  Serial.print(ay); Serial.print(" ");
  Serial.println(az);

  Serial.print("Gyro  [X Y Z]: ");
  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.println(gz);

  // ----- GPS -----
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  if (gps.location.isUpdated()) {
    Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
    Serial.print("Speed (km/h): "); Serial.println(gps.speed.kmph());
    Serial.print("Satellites: "); Serial.println(gps.satellites.value());
  }

  Serial.println("-----------------------------");
  delay(200); // update tiap 200ms
}
