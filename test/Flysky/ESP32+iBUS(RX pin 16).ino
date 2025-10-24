#include <IBusBM.h>

HardwareSerial IBusSerial(1);
IBusBM ibus;

void setup() {
  Serial.begin(115200);

  // RX = GPIO16, TX = tidak dipakai (-1)
  IBusSerial.begin(115200, SERIAL_8N1, 16, -1);
  ibus.begin(IBusSerial);
}

void loop() {
  ibus.loop();

  int ch1 = ibus.readChannel(0); // Roll
  int ch2 = ibus.readChannel(1); // Pitch
  int ch3 = ibus.readChannel(2); // Throttle
  int ch4 = ibus.readChannel(3); // Yaw
  int ch5 = ibus.readChannel(4); // Aux1
  int ch6 = ibus.readChannel(5); // Aux2

  Serial.print("CH1: "); Serial.print(ch1);
  Serial.print(" | CH2: "); Serial.print(ch2);
  Serial.print(" | CH3: "); Serial.print(ch3);
  Serial.print(" | CH4: "); Serial.print(ch4);
  Serial.print(" | CH5: "); Serial.print(ch5);
  Serial.print(" | CH6: "); Serial.println(ch6);

  delay(50);
}
