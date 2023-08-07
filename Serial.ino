#include <HardwareSerial.h>

int RX_PIN = 22;
int TX_PIN = 23;

void setup() {;
  Serial2.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop() {
  int camera_order = Serial2.read();
    if (camera_order == 0){
      brake();止まる
    }
    elif (camera_order  == 1){
      forward();//前へ
    }
    elif (camera_order == 2){
      leftturn();//右へ
    }
    elif (camera_order == 3){
      rightturn();//左へ
    else () {
      slow_rotating();//探す
    }

  //Serial2.println("Hello Raspberry pi");
  //delay(100);
}
