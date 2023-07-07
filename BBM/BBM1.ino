#include <Wire.h>                                                         //BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
float temp;
float pressure;
float humid;

#include <TinyGPS++.h>                                                     //GPS
TinyGPSPlus gps;

int cutparac = 23;          //切り離し用トランジスタのピン番号の宣言           //nicromewire
int outputcutsecond = 3;    //切り離し時の9V電圧を流す時間，単位はsecond

void setup(){
  
 Serial.begin(115200);                                                       //BME280
  bool status;
  status = bme.begin(0x76);  
  while (!status) {
    Serial.println("BME280 sensorが使えません");
    delay(1000);}
    
    Serial.begin(115200);                                                    //GPS
  Serial1.begin(9600,SERIAL_8N1,5,18); 
    
    Serial.begin(115200);                                                    //motor
  ledcSetup(0, 490, 8);
  ledcSetup(1, 490, 8);
  ledcSetup(2, 490, 8);
  ledcSetup(3, 490, 8);

  ledcAttachPin(32, 0);
  ledcAttachPin(33, 1);
  ledcAttachPin(26, 2);
  ledcAttachPin(25, 3);
  
   pinMode(cutparac, OUTPUT);      //切り離し用トランジスタの出力宣言           //nicromewire
    digitalWrite(cutparac, LOW);    //切り離し用トランジスタの出力オフ
    Serial.begin(115200);
}

void loop(){
 temp=bme.readTemperature();                                                  //BME280
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
  
   while (Serial1.available() > 0) {                                            //GPS
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {
      Serial.print("LAT:  "); Serial.println(gps.location.lat(), 9);
      Serial.print("LONG: "); Serial.println(gps.location.lng(), 9);
    }
  }
   //stoppage();                                                                //motor
  //delay(5000);
  forward();
  Serial2.println("forward now");
  delay(10000);
  rotating();
  Serial2.println("rotating now");
  delay(10000);
  reverse_rotating();
  Serial2.println("reverse_rotating now");
  delay(10000);
  
}

// 前進
void forward()
{
  ledcWrite(0, 0); // channel, duty
  ledcWrite(1, 5000);
  ledcWrite(2, 0);
  ledcWrite(3, 5000);
}
// 停止
void stoppage()
{
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}
// 回転
void rotating()
{
  ledcWrite(0, 0);
  ledcWrite(1, 5000);
  ledcWrite(2, 5000);
  ledcWrite(3, 0);
}
// 反回転
void reverse_rotating()
{
  ledcWrite(0, 5000);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 5000);

   delay(10000);           //nicromewire
   Serial.print("WARNING: 9v voltage on.\n");
   digitalWrite(cutparac, HIGH); //オン
   delay(outputcutsecond*1000);//十秒間電流を流す
   Serial.print("WARNING: 9v voltage off.\n");
   digitalWrite(cutparac, LOW); //オフ

}
