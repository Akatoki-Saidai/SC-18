#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;

float temp;
float pressure;
float humid;

void setup(){
  Serial.begin(115200);
  bool status;
  status = bme.begin(0x76);  
  while (!status) {
    Serial.println("BME280 sensorが使えません");
    delay(1000);
  }

}

void loop(){

}
