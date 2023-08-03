#include <DFRobot_QMC5883.h>
//calculation
#include <math.h>
#define rad2deg(a) ((a) / M_PI * 180.0) /*red -> deg*/
#define deg2rad(a) ((a) / 180.0 * M_PI) /*deg -> red*/

double heading;
double declinationAngle;
double headingDegrees;
double Angle_gy270;
double Angle_Goal;
double Angle_gps;
double Angle_heading;
double rrAngle, llAngle;
double Sum_headingDegrees;

double GOAL_lat = 35.860545000;
double GOAL_lng = 139.606940001;

double distance; //直進前後でゴールに近づいているかどうかを表す
double Pre_distance;
// variables___GY-271
double heading_data;
double heading_array[5];
double heading_sum = 0;
double omega;
double azimuth;
double delta_lng;
double gps_latitude;
double gps_longitude;


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
//右旋回
void leftturn()
{
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 255);
  ledcWrite(3, 0);
}

//左旋回
void rightturn()
{
  ledcWrite(0, 255);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

//回転
void rotating()
{
  ledcWrite(0, 0);
  ledcWrite(1, 140);
  ledcWrite(2, 140);
  ledcWrite(3, 0);
}

//ゆっくり回転
void slow_rotating()
{
  ledcWrite(0, 0);
  ledcWrite(1, 90);
  ledcWrite(2, 90);
  ledcWrite(3, 0);
}

//反回転
void reverse_rotating()
{
  ledcWrite(0, 140);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 140);
}


//緯度経度から距離を返す関数
double CalculateDis(double GOAL_lng, double GOAL_lat, double gps_longitude, double gps_latitude){
  GOAL_lng = deg2rad(GOAL_lng);
  GOAL_lat = deg2rad(GOAL_lat);

  gps_longitude = deg2rad(gps_longitude);
  gps_latitude = deg2rad(gps_latitude);

  double EarthRadius = 6378.137;

  // 目標地点までの距離を導出
  delta_lng = GOAL_lng - gps_longitude;

  distance = EarthRadius * acos(sin(gps_latitude) * sin(GOAL_lat) + cos(gps_latitude) * cos(GOAL_lat) * cos(delta_lng)) * 1000;

  return distance;
}

double desiredDistance = 0.0;

// 角度計算用の関数
double CalculateAngle(double GOAL_lng, double GOAL_lat, double gps_longitude, double gps_latitude)
{

  GOAL_lng = deg2rad(GOAL_lng);
  GOAL_lat = deg2rad(GOAL_lat);

  gps_longitude = deg2rad(gps_longitude);
  gps_latitude = deg2rad(gps_latitude);

  // 目標地点までの角度を導出
  delta_lng = GOAL_lng - gps_longitude;
  azimuth = rad2deg(atan2(sin(delta_lng), cos(gps_latitude) * tan(GOAL_lat) - sin(gps_latitude) * cos(delta_lng)));

  if (azimuth < 0)// azimuthを0°～360°に収める
  {
    azimuth += 360;
  }
  else if (azimuth > 360)
  {
    azimuth -= 360;
  }
  return azimuth;
}



void setup(){
    
  Serial.begin(115200);


  //Raspberry Pi
  Serial2.begin(19200,SERIAL_8N1, RX_PIN, TX_PIN);

//GPS
  Serial1.begin(9600,SERIAL_8N1,5,18); 
//GPS

//BNO055
  Wire.begin(21,22);
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

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

    
}

void loop() {
    while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);
    if (gps.location.isUpdated()){

    // Add your GPS-related code here, using the 'gps' object
  // BME280
  float temp, pressure, humid, altitude; // Assuming these variables are defined in BME280 sensor code, declare them here

  //BNO055
  // ジャイロセンサ値の取得と表示
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // 加速度センサ値の取得と表示
  // BNO055からセンサーデータを取得
  sensors_event_t event;
  bno.getEvent(&event);

  // Z軸の加速度を取得
  float accelZ = event.acceleration.z;
  // 磁力センサ値の取得と表示
  imu::Vector<3> magnetmetor = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

        gps_latitude = gps.location.lat();
        gps_longitude = gps.location.lng();



    while(accelZ <= 12.0) {
      //Serial2.println(gps.time);
      stoppage();
      delay(50);
    }
/*
    delay(1000);
    Serial.print("WARNING: 9v voltage on.\n");
    digitalWrite(cutparac, HIGH); //オン
    delay(outputcutsecond * 1000); //十秒間電流を流す
    Serial.print("WARNING: 9v voltage off.\n");
    digitalWrite(cutparac, LOW); //オフ
    */

    accel();
    delay(3000);
    brake();
    delay(3000);

    delay(1000);
      //Serial2.println(gps_time);
      Serial2.println("transition completed");
      Serial2.print("LAT(PHASE4_START):");
      Serial2.println(gps_latitude, 9);
      Serial2.print("LONG(PHASE4_START):");
      Serial2.println(gps_longitude, 9);
      Serial2.flush();

          int CurrentDistance = CalculateDis(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
          Serial2.print("CurrentDistance=");
          Serial2.println(CurrentDistance);

          if (desiredDistance >= CurrentDistance)
          {
            // カラーコーンとの距離が理想値よりも小さい場合は次のフェーズに移行する
            break;
          }else{
            delay(100);
            accel();
            Serial2.print("LAT:");
            Serial2.println(gps_latitude, 9);
            Serial2.print("LONG:");
            Serial2.println(gps_longitude, 9);
            Serial2.flush();
            delay(3000);
            forward();
            Serial2.print("LAT:");
            Serial2.println(gps_latitude, 9);
            Serial2.print("LONG:");
            Serial2.println(gps_longitude, 9);
            Serial2.flush();
            delay(1000);
            stoppage();
            // Goalまでの偏角を計算する
            Angle_Goal = CalculateAngle(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);

            Sum_headingDegrees = 0.0;
              for (int i = 0; i < 15; i++){
                delay(10);
                heading = event.orientation.x;
                declinationAngle = (-7.0 + (46.0 / 60.0)) / (180 / PI);
                heading += declinationAngle;
                if (heading < 0){
                  heading += 2 * PI;
                }
                if (heading > 2 * PI){
                  heading -= 2 * PI;
                }

                // Convert to degrees
                headingDegrees = heading * 180 / M_PI;

                if (headingDegrees < 0){
                  headingDegrees += 360;
                }

                if (headingDegrees > 360){
                  headingDegrees -= 360;
                }
                Sum_headingDegrees += headingDegrees;
              }
              Angle_gy270 = Sum_headingDegrees / 15;

              // どちらに回ればいいか計算
              rrAngle = -Angle_gy270 + Angle_Goal;
              if (rrAngle < 0){
                rrAngle += 360;
              }
              if (rrAngle > 360){
                rrAngle -= 360;
              }
              llAngle = Angle_gy270 - Angle_Goal;
              if (llAngle < 0){
                llAngle += 360;
              }
              if (llAngle > 360){
                llAngle -= 360;
              }

            if (rrAngle > llAngle){
              //反時計回り
              if (llAngle > 20){
                leftturn();
                delay(100);
                rotating();
                delay(400);
                stoppage();
              }
            }else{
              //時計回り
              if (rrAngle > 20){
                rightturn();
                delay(100);
                reverse_rotating();
                delay(400);
                stoppage();
              }
            }
          }
        
        break;
        }



        delay(1000000000);
  }
}
