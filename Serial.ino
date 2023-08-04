#include <HardwareSerial.h>

int RX_PIN = 22;
int TX_PIN = 23;

void setup() {
  Serial.begin(9600);
  Serial2.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("on");
}

void loop() {
  
  int camera_order = Serial2.parseInt();
  //int camera_order = Serial2.read();
  Serial.println(camera_order);

    /*
    if (camera_order == 0){
      ;
    } //stoppage
    elif (camera_order  == 1){
      ;
    } //forward
    elif (camera_order == 2){
      ;
    } //right
    elif (camera_order == 3){
      ;
    } //left*/

  Serial2.println("Hello Raspberry pi");
  delay(100);
}
