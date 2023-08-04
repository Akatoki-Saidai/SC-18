#include <HardwareSerial.h>
int RX_PIN = 22;
int TX_PIN = 23;
int phase = 0;
int phase_state = 0;

#include <TinyGPS++.h>
TinyGPSPlus gps;
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600,SERIAL_8N1,5,18);
  Serial2.begin(19200,SERIAL_8N1, RX_PIN, TX_PIN);
}
void loop() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {
      Serial.print("LAT:  "); Serial.println(gps.location.lat(), 9);
      Serial.print("LONG: "); Serial.println(gps.location.lng(), 9);
      Serial2.println("transition completed");
      Serial2.print("LAT(PHASE4_START):");
      Serial2.println(gps.location.lat(), 9);
      Serial2.print("LONG(PHASE4_START):");
      Serial2.println(gps.location.lng(), 9);
      Serial2.println();
      delay(2000);
      Serial2.flush();
          
    }
  }
}