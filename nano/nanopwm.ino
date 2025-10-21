#include <Servo.h>

Servo myServo;       // Servo atau ESC
int servoPin = 13;   // Pin Nano yang digunakan
int servoPos = 90;   // Posisi awal servo (0-180)
int escSpeed = 0;    // Kecepatan ESC (0-180, map sesuai throttle)

void setup() {
  myServo.attach(servoPin); // Hubungkan servo/ESC ke pin 13
  myServo.write(servoPos);  // Set posisi awal servo
  Serial.begin(9600);
  Serial.println("Servo & ESC tester ready!");
}

void loop() {
  // Contoh menggerakkan servo secara bertahap
  for(servoPos = 0; servoPos <= 180; servoPos += 1) {
    myServo.write(servoPos);
    delay(15); // Delay untuk gerakan halus
  }
  for(servoPos = 180; servoPos >= 0; servoPos -= 1) {
    myServo.write(servoPos);
    delay(15);
  }

  // Contoh kontrol ESC / motor brushless
  for(escSpeed = 0; escSpeed <= 180; escSpeed += 5) {
    myServo.write(escSpeed); // ESC menerima sinyal PWM sama seperti servo
    delay(200);
  }
  for(escSpeed = 180; escSpeed >= 0; escSpeed -= 5) {
    myServo.write(escSpeed);
    delay(200);
  }
}
