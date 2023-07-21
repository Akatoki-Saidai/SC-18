#include <HardwareSerial.h>

int RX_PIN = 22;
int TX_PIN = 23;

void setup() {
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop() {
  Serial1.println("Hello Raspberry pi");
  delay(100);
}