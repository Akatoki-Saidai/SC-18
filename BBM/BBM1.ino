//for BME280
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;

float temp;
float pressure;
float humid;


//for GPS
#include <TinyGPS++.h>
TinyGPSPlus gps;


void setup(){
  //for BME280
  Serial.begin(115200);
  bool status;
  status = bme.begin(0x76);  
  while (!status) {
    Serial.println("BME280 sensorが使えません");
    delay(1000);
  }
  

  //for GPS
  Serial.begin(115200);
  Serial1.begin(9600,SERIAL_8N1,5,18);
}

void loop(){
  //for BME280
  temp=bme.readTemperature();
  pressure=bme.readPressure() / 100.0F;
  humid=bme.readHumidity();
  Serial.print("温度 ;");
  Serial.print(temp);
  Serial.println(" °C");
   
  Serial.print("気圧 ;");
  Serial.print(pressure);
  Serial.println(" hPa");
  Serial.print("湿度 ;");
  Serial.print(humid);
  Serial.println(" %");
  Serial.println();
  delay(1000);

  //for GPS
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {
      Serial.print("LAT:  "); Serial.println(gps.location.lat(), 9);
      Serial.print("LONG: "); Serial.println(gps.location.lng(), 9);
    }
  }

}
