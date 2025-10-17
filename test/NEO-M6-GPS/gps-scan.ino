#include <TinyGPS++.h>

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // gunakan UART1 ESP32

#define GPS_RX 16  // RX pin ESP32 ke TX GPS
#define GPS_TX 17  // TX pin ESP32 ke RX GPS (optional)

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("\n=== SCAN GPS NEO-6M ===");
  Serial.println("Menunggu data GPS...");
}

void loop() {
  // Baca data dari GPS module
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Jika lokasi terupdate, tampilkan datanya
  if (gps.location.isUpdated()) {
    Serial.println("\n=== DATA GPS ===");
    Serial.print("Latitude   : "); Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude  : "); Serial.println(gps.location.lng(), 6);
    Serial.print("Kecepatan  : "); Serial.print(gps.speed.kmph()); Serial.println(" km/h");
    Serial.print("Arah       : "); Serial.print(gps.course.deg()); Serial.println("°");
    Serial.print("Ketinggian : "); Serial.print(gps.altitude.meters()); Serial.println(" m");
    Serial.print("Satellites : "); Serial.println(gps.satellites.value());
    Serial.print("HDOP       : "); Serial.println(gps.hdop.hdop());
    Serial.println("--------------------------");
  }

  delay(500);
}
