#include <DFRobot_QMC5883.h>
//calculation
#include <math.h>
#define rad2deg(a) ((a) / M_PI * 180.0) /*red -> deg*/
#define deg2rad(a) ((a) / 180.0 * M_PI) /*deg -> red*/

double heading;
double declinationAngle;
double headingDegrees;
double Angle_north;
double Angle_Goal;
double Angle_gps;
double Angle_heading;
double rrAngle, llAngle;
double Sum_headingDegrees;

double GOAL_lat = 35.86512;
double GOAL_lng = 139.60740;

double distance; //直進前後でゴールに近づいているかどうかを表す
double Pre_distance;
double heading_data;
double heading_array[5];
double heading_sum = 0;
double omega;
double azimuth;
double delta_lng;
double altitude0;
int camera_order;
int phase = 0;
int phase_state = 0;
int altitude_phasestate = 0; //地上から待機フェーズに移行するための変数

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


//BNO055
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
//BNO055


//GPS
#include <TinyGPS++.h>
//#include <HardwareSerial.h>
TinyGPSPlus gps;
//HardwareSerial SerialGPS(1);
//GPS

double gps_latitude = gps.location.lat();
double gps_longitude = gps.location.lng();

//Raspberry Pi 通信
#include <HardwareSerial.h>
int RX_PIN = 22;
int TX_PIN = 23;


//bnicromewire
int cutparac = 23;          //切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 3;    //切り離し時の9V電圧を流す時間，単位はsecond


//motor
void accel()
{
for (int i = 0; i < 240; i = i + 6)
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
  for (int i = 240; i > 0; i = i - 6)
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
void reverse_rotating()
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
void rotating()
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

double desiredDistance = 5.0;

// 角度計算用の関数
double CalculateAngle(double GOAL_lng, double GOAL_lat, double gps_lng, double gps_lat)
{
  double degrees = (atan2((gps_lng - GOAL_lng) * 1.23, (gps_lat - GOAL_lat)) * 57.3 + 180);
  return degrees;
}

double accelXYZ (double accelX,double accelY,double accelZ){
  double E_pow_accelXYZ = sqrt((pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2)));
  return E_pow_accelXYZ;
}



void setup(){
    
  Serial.begin(115200);


  //Raspberry Pi
  Serial2.begin(19200,SERIAL_8N1, RX_PIN, TX_PIN);

  //GPS
  Serial1.begin(9600,SERIAL_8N1,5,18); 

    // BME280
  bool status = bme.begin(0x76);
  if (!status) {
    Serial.println("BME280 sensorが使えません");
    while (1);
  }
  altitude0 = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.println(altitude0);

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

  //nicromewire
  pinMode(cutparac, OUTPUT);      //切り離し用トランジスタの出力宣言
  digitalWrite(cutparac, LOW);    //切り離し用トランジスタの出力オフ
    
}

void loop() {
  
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()){
      //こっからスタート

      switch (phase){
        case 0:
          if(altitude_phasestate == 0){
            altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
            Serial.print("altitude0 :");
            Serial.println(altitude0);
            Serial.print("altitude :");
            Serial.println(altitude);
            delay(100);
            if(altitude > altitude0 + 1.0){
              altitude_phasestate = 1;
            }
          }
          if(altitude_phasestate == 1){
            Serial.println("stand-by phase");
            Serial.println("accelZ");
            // BNO055からセンサーデータを取得
            imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
            double accelZ = accelermetor.z();
            double accelX = accelermetor.x();
            double accelY = accelermetor.y();
        
            Serial.println(accelX);
            Serial.println(accelY);
            Serial.println(accelZ);
            //Serial2.println(gps.time);
            stoppage();
            if(accelXYZ(accelX, accelY, accelZ) >= 20.0){
              phase_state = 1;
              altitude_phasestate = 2;
              Serial.println("falling!");
            }
          }
          if(phase_state == 1){
            //高度測定
            altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
            if(altitude < altitude0 + 1.0){
              Serial.println("Go to long phase");
              delay(5000);
  
              phase_state = 2;
              //ニクロム線を切る
              /*
              delay(1000);
              Serial.print("WARNING: 9v voltage on.\n");
              digitalWrite(cutparac, HIGH); //オン
              delay(outputcutsecond * 1000); //十秒間電流を流す
              Serial.print("WARNING: 9v voltage off.\n");
              digitalWrite(cutparac, LOW); //オフ
              */
            
              //初期前進
              accel();
              delay(2000);
              brake();
              delay(2000);
              phase = 1;
            }
          }
        break;

      
        case 1:
          if(phase_state == 2){
            Serial.print("LAT:");
            Serial.println(gps.location.lat(), 9);
            Serial.print("LONG:");
            Serial.println(gps.location.lng(), 9);
            Serial.flush();
            Serial2.println("transition completed");
            Serial2.print("LAT(PHASE4_START):");
            Serial2.println(gps_latitude, 9);
            Serial2.print("LONG(PHASE4_START):");
            Serial2.println(gps_longitude, 9);
            Serial2.flush();
            Serial.println("calibration start");
            imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            rotating();
            delay(5000);
            phase_state = 3;
          }
          // Add your GPS-related code here, using the 'gps' object
          gps_latitude = gps.location.lat();
          gps_longitude = gps.location.lng();
      

          if(phase_state == 3){
            gps_latitude = gps.location.lat();
            gps_longitude = gps.location.lng();
            int CurrentDistance = CalculateDis(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
            Serial.print("CurrentDistance=");
            Serial.println(CurrentDistance);
            if (desiredDistance >= CurrentDistance){
              // カラーコーンとの距離が理想値よりも小さい場合は次のフェーズに移行する
              Serial.println("Go to short phase");
              phase = 2;
              phase_state = 3;
            }else{
              Serial.println("Continue long_phase");
              delay(100);
              /*
              accel();
              delay(2000);
              Serial.print("LAT:");
              Serial.println(gps.location.lat(), 9);
              Serial.print("LONG:");
              Serial.println(gps.location.lng(), 9);
              Serial.flush();
              Serial2.print("LAT:");
              Serial2.println(gps.location.lat(), 9);
              Serial2.print("LONG:");
              Serial2.println(gps.location.lng(), 9);
              Serial2.flush();
              */
              brake();
              delay(2000);
              Serial.print("LAT:");
              Serial.println(gps.location.lat(), 9);
              Serial.print("LONG:");
              Serial.println(gps.location.lng(), 9);
              Serial.flush();
              Serial2.print("LAT:");
              Serial2.println(gps.location.lat(), 9);
              Serial2.print("LONG:");
              Serial2.println(gps.location.lng(), 9);
              Serial2.flush();
              delay(100);
              stoppage();
              // Goalまでの偏角を計算する
              gps_latitude = gps.location.lat();
              gps_longitude = gps.location.lng();
              Angle_Goal = CalculateAngle(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
              Serial.println("degrees calculationning");
              Serial.print("Angle_Goal :");
              Serial.println(Angle_Goal);
              //北基準機体角度Angle_north
              imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
              Angle_north = euler.x();
              //80度右向いてる
              if(Angle_north <= 280){
                Angle_north + 80;
              }else{
                Angle_north - 280;
              }
              Serial.print("Angle_north :");
              Serial.println(Angle_north);
              
              // どちらに回ればいいか計算
              rrAngle = -Angle_north + Angle_Goal;
              if (rrAngle < 0){
                rrAngle += 360;
              }
              if (rrAngle > 360){
                rrAngle -= 360;
              }
              llAngle = Angle_north - Angle_Goal;
              if (llAngle < 0){
                llAngle += 360;
              }
              if (llAngle > 360){
                llAngle -= 360;
              }
              Serial.print("rrAngle :");
              Serial.println(rrAngle);
              Serial.print("llAngle :");
              Serial.println(llAngle);

              if (rrAngle > llAngle){
                //反時計回り
                if (llAngle > 20){
                  Serial.println("left turn");
                  leftturn();
                  delay(100);
                  rotating();
                  delay(400);
                  stoppage();
                  delay(1000);
                }
              }else{
                //時計回り
                if (rrAngle > 20){
                  Serial.println("right turn");
                  rightturn();
                  delay(100);
                  reverse_rotating();
                  delay(400);
                  stoppage();
                  delay(1000);
                }
              }
            }
          }
        break;


        case 2:

          if(phase_state == 3){
            Serial2.println("CameraStart");
            Serial.println("Continue short phase");
            int camera_order = Serial2.read();
            Serial.print("camera_order :");
            Serial.println(camera_order);
             if (camera_order == 0){
               brake();
               Serial.println("GOAL GOAL GOAL");//コーンの面積が閾値を超えた
               Serial2.println("GOAL GOAL GOAL");
               delay(2000);
             }else if (camera_order  == 1){
               forward();
               delay(2000);
             }else if (camera_order == 2){
               rightturn();
               delay(100);
             }else if (camera_order == 3){
               leftturn();
               delay(100);
             }else{
               slow_rotating();
               delay(100)
             }
           }
        break;
        /*
        case 3:
          stoppage();
          delay(10);
        break;
        */
      }
    }
  }
}
