//calculation
#include <math.h>
#define red2deg(a) ((a) / M_PI * 180.0) /*red -> deg*/
#define deg2red(a) ((a) / 180.0 * M_PI) /*deg -> red*/

//BME280
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//1013.25は地球の海面上の大気圧の平均値(1気圧)です。
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

float temp;
float pressure;
float humid;
float altitude;//高度
//BME280


//BNO055
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
//BNO055


//GPS
#include <TinyGPS++.h>
#include <HardwareSerial.h>
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
//GPS


//Raspberry Pi 通信
#include <HardwareSerial.h>
int RX_PIN = 22;
int TX_PIN = 23;

int phase = 0;
int phase_state = 0;

//bnicromewire
int cutparac = 23;          //切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 3;    //切り離し時の9V電圧を流す時間，単位はsecond
//enicromewire




//motor
void accel()
{
for (int i = 10; i < 240; i = i + 4)
  {
    ledcWrite(0, 0);
    ledcWrite(1, i);
    ledcWrite(2, 0);
    ledcWrite(3, i);
    delay(50); // accelではdelay使う
  }
}
//brake
void brake()
{
  for (int i = 240; i > 0; i = i - 4)
  {
    ledcWrite(0, 0);
    ledcWrite(1, i);
    ledcWrite(2, 0);
    ledcWrite(3, i);
    delay(50); // stoppingではdelay使う
  }
}
// 前進
void forward()
{
  ledcWrite(0, 0); // channel, duty
  ledcWrite(1, 10000);
  ledcWrite(2, 0);
  ledcWrite(3, 10000);
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
}
//motor




void setup(){
    
  Serial.begin(115200);

  // BME280
  bool status = bme.begin(0x76);
  if (!status) {
    Serial.println("BME280 sensorが使えません");
    while (1);
  }
  //BME280
    

//GPS
  Serial1.begin(9600,SERIAL_8N1,5,18); 
//GPS



//bmotor
  ledcSetup(0, 490, 8);
  ledcSetup(1, 490, 8);
  ledcSetup(2, 490, 8);
  ledcSetup(3, 490, 8);

  ledcAttachPin(32, 0);
  ledcAttachPin(33, 1);
  ledcAttachPin(26, 2);
  ledcAttachPin(25, 3);
//motor



//nicromewire
  pinMode(cutparac, OUTPUT);      //切り離し用トランジスタの出力宣言
  digitalWrite(cutparac, LOW);    //切り離し用トランジスタの出力オフ


//nicromewire


//BNO055
  Wire.begin(21,22);
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
//BNO055

//BNO055
  /* Display the current temperature */
  /*
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(false);
  

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  bno055ticker.attach_ms(BNO055interval, get_bno055_data);
  */
//BNO055
    
}

void loop(){
    //BME280
  temp=bme.readTemperature();
  pressure=bme.readPressure() / 100.0F;
  humid=bme.readHumidity();
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  //BNO055
   // ジャイロセンサ値の取得と表示
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // 加速度センサ値の取得と表示
  imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
   // 磁力センサ値の取得と表示
  imu::Vector<3> magnetmetor = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  
    while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
      while(accelermetor.z-9.8 >= 0){
        Serial2.println(gps_time);
        stoppage();
        delay(50);
      }
        delay(10000);
    Serial.print("WARNING: 9v voltage on.\n");
    digitalWrite(cutparac, HIGH); //オン
    delay(outputcutsecond*1000);//十秒間電流を流す
    Serial.print("WARNING: 9v voltage off.\n");
    digitalWrite(cutparac, LOW); //オフ

    accel();
    delay(3000);

      
    

      
      

      
    }
}
