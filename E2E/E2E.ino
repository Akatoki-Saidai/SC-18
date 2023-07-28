//calculation
#include <math.h>
#define red2deg(a) ((a) / M_PI * 180.0) /*red -> deg*/
#define deg2red(a) ((a) / 180.0 * M_PI) /*deg -> red*/

//BME280
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

float temp;
float pressure;
float humid;
float altitude;

//BNO055
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
int CalibrationCounter = 1;  //キャリブレーションで取得したデータの個数
double declinationAngle; //補正用
double heading;  //弧度法
double headingDegrees;  //度数法
double Sum_headingDegrees;  //連続15個のheadingDegreesの和
double Angle_gy271;  //連続15個の平均値（度数法）


//GPS
#include <TinyGPS++.h>
#inlcude <HardwareSerial.h>
TinyGPSlus gps;
HardwareSerial SerialGPS(1);
double gps_latitude, gps_longitude, gps_velocity;
int gps_time;
double GOAL_lat = ;
double GOAL_lng = ;

//nicromewire
int cutparac = 23;
int outputcutsecond = 3;

//motor
//forward
void forward(){
  ledcWrite(0,0);
  ledcWrite(1,5000);
  ledcWrite(2,0);
  ledcWrite(3,5000);
}
//accel
void accel()
{
  for (int i = 0; i < 2500; i = i + 10)
  {
    ledcWrite(0, 0);
    ledcWrite(1, i);
    ledcWrite(2, 0);
    ledcWrite(3, i);
    delay(80); // accelではdelay使う
  }
}
//brake
void brake()
{
  for (int i = 2500; i > 0; i = i - 10)
  {
    ledcWrite(0, 0);
    ledcWrite(1, i);
    ledcWrite(2, 0);
    ledcWrite(3, i);
    delay(80); // stoppingではdelay使う
  }
}
//stop
void stoppage(){
  ledcWrite(0,0);
  ledcWrite(1,0);
  ledcWrite(2,0);
  ledcWrite(3,0);
}
//rotate
void rotating(){
  ledcWrite(0,0);
  ledcWrite(1,5000);
  ledcWrite(2,5000);
  ledcWrite(3,0);
}
//reverse_rotate
void reverse_rotating(){
  ledcWrite(0,5000);
  ledcWrite(1,0);
  ledcWrite(2,0);
  ledcWrite(3,5000);
}

//Raspberry Pi 通信
#include <HardwareSerial.h>
int RX_PIN = 22;
int TX_PIN = 23;

int phase = 0;
int phase_state = 0;

// for phase1
int mode_comparison = 0;
int mode_to_bme = 0;
int count1 = 0;
int count3 = 0;
double altitude_sum_bme = 0;
double altitude,previous_altitude,current_altitude;
unsigned long previous_millis,current_millis;

// for phase2
int type = 1;
int yeah = 1;
int type_state = 0;
double time3_2; // 時間に関するもの
double Accel[6];                  // 計測した値をおいておく変数
double Altitude[6];               // 高度
double Preac, differ1, Acsum, Acave, RealDiffer1;
double Preal, differ2, Alsum, Alave, RealDiffer2;
int i_3 = 0;
int j_3 = 0;
double RealDiffer;

// for phase3
int cutparac = 23;          // 切り離し用トランジスタのピン番号の宣言
int outputcutsecond = 3;    // 切り離し時の9V電圧を流す時間，単位はsecond

// for phase4
double desiredDistance = 10.0; //遠距離フェーズから中距離フェーズに移行する距離
double CurrentDistance;
double Angle_Goal,rrAngle,llAngle;

// for phase5
double sum_latitude,sum_longitude;
unsigned long nowmillis;
int sum_count = 0;
int EPSILON = 30;

// for phase6
double current_distance,previous_distance,distance1,distance2;
unsigned long current_Millis,time1,time2;
int phase_5 = 1;
int count = 0;
int accel_count = 1;


//緯度経度から距離を返す関数
double CalculateDis(double GOAL_lng, double GOAL_lat, double gps_longitude, double gps_latitude){
  GOAL_lng = deg2red(GOAL_lng);
  GOAL_lat = deg2red(GOAL_lat);

  gps_longitude = deg2rad(gps_longitude);
  gps_latitude = deg2rad(gps_latitude);

  double EarthRadius = 6378.137;

  // 目標地点までの距離を導出
  delta_lng = GOAL_lng - gps_longitude;

  distance = EarthRadius * acos(sin(gps_latitude) * sin(GOAL_lat) + cos(gps_latitude) * cos(GOAL_lat) * cos(delta_lng)) * 1000;

  return distance;
}

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

//BME280
  bool status = bme.begin(0x76);
  if(!status) {
    Serial.println("BME280 sensor can't be used);
    while(1);
  }


//GPS
    Serial1.begin(9600,SERIAL_8N1,5,18);

//motor
  ledcSetup(0,490,8);
  ledxSetup(1,490,8);
  ledcSetup(2,490,8);
  ledcSetup(3,490,8);

  ledcAttachPin(32,0);
  ledcAttachPin(33,1);
  ledcAttachPin(26,2);
  ledcAttachPin(25,3);


//nicromewire
  pinMode(cutparac, OUTPUT);      //切り離し用トランジスタの出力宣言
  digitalWrite(cutparac, LOW);    //切り離し用トランジスタの出力オフ

//BNO055
  Wire.begin(21,22);
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

    /*
//BNO055
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  bno055ticker.attach_ms(BNO055interval, get_bno055_data);
  */
//BNO055
    
}


void loop(){
  if (Serial1.available() > 0)
  { //これないとGPS使えない
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated())
    { //この中でgpsのデータを使わないと，うまく値が表示されない
      if (timeCounter1 > 0)
      {
        portENTER_CRITICAL(&timerMux);
        timeCounter1--;
        portEXIT_CRITICAL(&timerMux);

        //起動時刻の更新
        unsigned long currentMillis = millis();

        //センサー値のアップデート
        mySensor.accelUpdate();

        //センサー値取得
        // BMP180
        Temperature = bme.readTemperature();
        Pressure = bme.readPressure();
        if (mySensor.accelUpdate() == 0)
        {
          accelX = mySensor.accelX() + 0;
          accelY = mySensor.accelY() + 0;
          accelZ = mySensor.accelZ() + 0;
          accelSqrt = mySensor.accelSqrt();
        }

        altitude = bme.readAltitude();


        /*gps_latitude = gps.location.lat();
        gps_longitude = gps.location.lng();*/

        while(sum_count <3)
        {
          // GPS
          char c = Serial1.read(); // GPSチップからのデータを受信
          gps.encode(c);           // GPSチップからのデータの整形
          sum_latitude += gps.location.lat();
          sum_longitude += gps.location.lng();       
          sum_count++;
        }
        sum_count = 0;
        gps_latitude = sum_latitude/3;
        gps_longitude = sum_longitude/3;
        sum_latitude = 0.0;
        sum_longitude = 0.0;

        gps_time = gps.time.value();
        gps_velocity = gps.speed.mps();


        Serial.print(gps_time);
        Serial.print(",");
        Serial.print(gps_latitude, 9);
        Serial.print(",");
        Serial.print(gps_longitude, 9);
        Serial.print(",");
        Serial.print(gps_velocity);
        Serial.print(",");
        Serial.print(Temperature);
        Serial.print(",");
        Serial.print(Pressure);
        Serial.print(",");
        Serial.print(accelX);
        Serial.print(",");
        Serial.print(accelY);
        Serial.print(",");
        Serial.print(accelZ);
        Serial.print(",");
        Serial.print(heading);
        Serial.print(",");
        Serial.print(headingDegrees);
        Serial.print(",");

        //BNO055
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
  /*
  // キャリブレーションのステータスの取得と表示
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIB Sys:");
  Serial.print(system, DEC);
  Serial.print(", Gy");
  Serial.print(gyro, DEC);
  Serial.print(", Ac");
  Serial.print(accel, DEC);
  Serial.print(", Mg");
  Serial.print(mag, DEC);
  */
  
  // ジャイロセンサ値の取得と表示
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print(" 　Gy_xyz:");
  Serial.print(gyroscope.x());
  Serial.print(", ");
  Serial.print(gyroscope.y());
  Serial.print(", ");
  Serial.print(gyroscope.z());
  
  
  // 加速度センサ値の取得と表示
  imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print(" 　Ac_xyz:");
  Serial.print(accelermetor.x());
  Serial.print(", ");
  Serial.print(accelermetor.y());
  Serial.print(", ");
  Serial.print(accelermetor.z());

 
  // 磁力センサ値の取得と表示
  imu::Vector<3> magnetmetor = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  Serial.print(" 　Mg_xyz:");
  Serial.print(magnetmetor .x());
  Serial.print(", ");
  Serial.print(magnetmetor .y());
  Serial.print(", ");
  Serial.print(magnetmetor .z());
  
  
  /*
  // センサフュージョンによる方向推定値の取得と表示
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print(" 　DIR_xyz:");
  Serial.print(euler.x());
  Serial.print(", ");
  Serial.print(euler.y());
  Serial.print(", ");
  Serial.print(euler.z());
*/
  /*
    // センサフュージョンの方向推定値のクオータニオン
    imu::Quaternion quat = bno.getQuat();
    Serial.print("qW: ");
    Serial.print(quat.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat.x(), 4);
    Serial.print(" qY: ");
    Serial.print(quat.y(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat.z(), 4);
    Serial.print("\t\t");
  */

  Serial.println();
  delay(1000);

        //各フェーズごとの記述
        switch (phase)
        {

          //########## 待機フェーズ ##########
        case 1:
          if (phase_state != 1)
          {
            //待機フェーズに入ったとき１回だけ実行したいプログラムを書く
            Serial.write("Phase1: transition completed\n"); // 地上局へのデータ送信
            // LogDataの保存
            Serial2.println(gps_time);
            SErial2.println("Phase1: transition completed");
            SErial2.flush();

            phase_state = 1;
          }
          if (mode_to_bme == 0)
          {
            if (accelZ < -2)
            { //落下開始をまずMPUで判定
              altitude_sum_BNO += altitude;
              count1++;
              if (count1 == 1)
              {
                count1 = 0;
                Serial2.println(gps_time);
                Serial2.println("FALL STARTED(by MPU)\n");
                Serial2.flush();
                mode_to_bme = 1;
              }
            }
          }
          else
          {
            switch (mode_comparison)
            { //落下開始をBMPで判定
            case 0:
              previous_millis = millis();
              altitude_sum_bme += altitude;
              count3++;
              if (count3 == 5)
              {
                previous_altitude = altitude_sum_bme / 5;
                altitude_sum_bme = 0;
                count3 = 0;
                mode_comparison = 1;
              }
              break;
            case 1: // 500ms後
              current_millis = millis();
              if (current_millis - previous_millis >= 500)
              {
                altitude_sum_bme += altitude;
                count3++;
                if (count3 == 5)
                {
                  current_altitude = altitude_sum_bme / 5;
                  Serial2.println(currentMillis);
                  Serial2.println("current_altitude - previous_altitude = \n");
                  Serial2.println(current_altitude - previous_altitude);
                  if (current_altitude - previous_altitude <= -1.0)
                  {
                    Serial2.println("FALL STARTED(by BMP)\n");
                    Serial2.flush();
                    phase = 2;
                  }
                  else
                  {
                    altitude_sum_bme = 0;
                    count3 = 0;
                    mode_comparison = 0;
                  }
                }
              }
              break;
            }
          }
          break;
        //########## 降下フェーズ ##########
        case 2:
          if (phase_state != 2)
          {
            //降下フェーズに入ったとき１回だけ実行したいプログラムを書く
            Serial.write("Phase2: transition completed\n"); //地上局へのデータ送信

            // LogDataの保存
            Serial2.println(gps_time);
            Serial2.println("Phase2: transition completed");
            Serial2.flush();

            i_3 = 0;
            j_3 = 0;
            Preac = 0; // 1個前の加速度を記憶
            Preal = 0;
            differ1 = 0.1;   //加速度　移動平均の差
            differ2 = 0.5;   //高度　　移動平均の差
            Acave = 0;       //加速度　5個の平均値
            Alave = 0;       //高度　　5個の平均値
            Acsum = 0;       //加速度　5個の合計値
            Alsum = 0;       //高度　　5個の合計値
            RealDiffer1 = 0; // 1秒前との差を記憶する
            RealDiffer2 = 0;

            phase_state = 2;
          }

          if (yeah == 1)
          { //データを初めから五個得るまで
            Accel[i_3] = accelSqrt;
            Altitude[i_3] = altitude;
            i_3 = i_3 + 1;
            if (i_3 == 6)
            { // 5個得たらその時点での平均値を出し，次のフェーズへ
              yeah = 2;
              i_3 = 0; // iの値をリセット
              for (j_3 = 1; j_3 < 6; j_3++)
              { // j_3=0の値は非常に誤差が大きいので1から
                Acsum = Acsum + Accel[j_3];
                Alsum = Alsum + Altitude[j_3];
              }
              Acave = Acsum / 5;
              Alave = Alave / 5;
              time3_2 = currentMillis;
            }
          }
          else
          {
            Preac = Acave;
            Preal = Alave;
            Accel[i_3] = accelSqrt;
            Altitude[i_3] = altitude;
            for (j_3 = 0; j_3 < 5; j_3++)
            {
              Acsum = Acsum + Accel[j_3];
              Alsum = Alsum + Altitude[j_3];
            }
            Acave = Acsum / 5;
            Alave = Alsum / 5;
            RealDiffer1 = Preac - Acave;
            RealDiffer2 = Preal - Alave;
            if (i_3 == 5)
            {
              i_3 = 0;
              Acsum = 0;
              Alsum = 0;
            }
            else
            {
              i_3 = i_3 + 1;
              Acsum = 0;
              Alsum = 0;
            }
            if (currentMillis - time3_2 > 1000)
            {
              if (RealDiffer1 < differ1)
              { //移動平均が基準以内の変化量だった時
                phase = 3;
              }
              else if (RealDiffer2 < differ2)
              {
                phase = 3;
              }
              else
              {
                time3_2 = currentMillis;
              }
            }
          }

          break;
        //########## 分離フェーズ ##########
        case 3:
          if (phase_state != 3)
          {
            //分離フェーズに入ったとき１回だけ実行したいプログラムを書く
            Serial.write("Phase3: transition completed\n");
            // LogDataの保存
            Serial2.println(gps_time);
            Serial2.println("Phase3: transition completed");
            Serial2.flush();
            phase_state = 3;
            time3_1 = currentMillis;                    // phase3　開始時間の保存
            St_Time = time3_1 + outputcutsecond * 1000; //基準時間
            Serial.write("WARNING: The cut-para code has been entered.\n");
            digitalWrite(cutparac, HIGH); //オン
            Serial.write("WARNING: 9v voltage is output.\n");
            // LogDataの保存
            Serial2.println(currentMillis);
            Serial2.println("9v voltage is output");
            Serial2.flush();
          }

          if (currentMillis > St_Time)
          {                              //電流を流した時間が基準時間を超えたら
            digitalWrite(cutparac, LOW); //オフ
            Serial.write("WARNING: 9v voltage is stop.\n");
            Serial2.println(currentMillis);
            Serial2.println("WARNING: 9v voltage is stop.\n");
            Serial2.flush();
            phase = 4;
          }

          break;

        //########## 遠距離探索フェーズ ##########
        case 4:
        {
          if (phase_state != 4){
            //遠距離探索フェーズに入ったとき１回だけ実行したいプログラムを書く
            Serial.write("Phase4: transition completed\n");
            Serial.print("LAT(PHASE4_START):");
            Serial.println(gps_latitude, 9);
            Serial.print("LONG(PHASE4_START):");
            Serial.println(gps_longitude, 9);

            // LogDataの保存
            Serial2.println(gps_time);
            Serial2.println("Phase4: transition completed");
            Serial2.print("LAT(PHASE4_START):");
            Serial2.println(gps_latitude, 9);
            Serial2.print("LONG(PHASE4_START):");
            Serial2.println(gps_longitude, 9);
            Serial2.flush();

            phase_state = 4;
          }

          CurrentDistance = CalculateDis(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
          Serial.print("CurrentDistance=");
          Serial.println(CurrentDistance);

          if (desiredDistance >= CurrentDistance)
          {
            // カラーコーンとの距離が理想値よりも小さい場合は次のフェーズに移行する
            phase = 0;
          }else{
            delay(100);
            accel();
            forward();
            delay(1000);
            stopping();
            // Goalまでの偏角を計算する
            Angle_Goal = CalculateAngle(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);

            Sum_headingDegrees = 0.0;
              for (i = 0; i < 15; i++){
                delay(10);
                Vector norm = compass.readNormalize();
                heading = atan2(norm.YAxis, norm.XAxis);
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


        //########## 中距離探索フェーズ ##########
        case 0:
        {
          if (phase_state != 0){
            //中距離探索フェーズに入ったとき１回だけ実行したいプログラムを書く
            Serial.write("Phase0: transition completed\n");
            Serial.print("LAT(PHASE0_START):");
            Serial.println(gps_latitude, 9);
            Serial.print("LONG(PHASE0_START):");
            Serial.println(gps_longitude, 9);

            // LogDataの保存
            Serial2.println(gps_time);
            Serial2.println("Phase0: transition completed");
            Serial2.print("LAT(PHASE0_START):");
            Serial2.println(gps_latitude, 9);
            Serial2.print("LONG(PHASE0_START):");
            Serial2.println(gps_longitude, 9);
            Serial2.flush();

            phase_state = 0;
          }
          nowmillis = millis();
          sum_latitude = 0.0;
          sum_longitude = 0.0;
          sum_count = 0;
          while(millis() - nowmillis < 3000){
            // GPS
            char c = Serial1.read(); // GPSチップからのデータを受信
            gps.encode(c);           // GPSチップからのデータの整形
            sum_latitude += gps.location.lat();
            sum_longitude += gps.location.lng();       
            sum_count++;
          }
          gps_latitude = sum_latitude/sum_count;
          gps_longitude = sum_longitude/sum_count;

          Serial.print(gps_latitude, 9);
          Serial.print(",");
          Serial.print(gps_longitude, 9);
          Serial.print(",");

          CurrentDistance = CalculateDis(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
          Serial.print("CurrentDistance=");
          Serial.println(CurrentDistance);

          ultra_distance = sr04.Distance();
          if (1.3 >= CurrentDistance || (ultra_distance < 600 && ultra_distance != 0)){
            // カラーコーンとの距離が理想値よりも小さい場合は次のフェーズに移行する
            phase = 5;
          }else{
            Angle_Goal = CalculateAngle(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);

            // ゆっくり回転開始
            slow_rotating();

            Vector norm = compass.readNormalize();
            heading = atan2(norm.YAxis, norm.XAxis);
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
  
            while(fabs(headingDegrees - Angle_Goal)>10){
              Vector norm = compass.readNormalize();
              heading = atan2(norm.YAxis, norm.XAxis);
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
             }
             stoppage();

            //少し進む(ここにお願い!)
            delay(100);
            forward();
            delay(1000);
            stopping();
        }
        break;}
        

        //########## 近距離探索フェーズ ##########
        case 5:
        {
          if (phase_state != 5)
          {
            //近距離探索フェーズに入ったとき１回だけ実行したいプログラムを書く
            Serial.write("Phase5: transition completed\n"); // 地上局へのデータ
            Serial.print("LAT(PHASE5_START):");
            Serial.println(gps_latitude, 9);
            Serial.print("LONG(PHASE5_START):");
            Serial.println(gps_longitude, 9);

            // LogDataの保存
            Serial2.println(gps_time);
            Serial2.println("Phase5: transition completed");
            Serial2.print("LAT(PHASE5_START):");
            Serial2.println(gps_latitude, 9);
            Serial2.print("LONG(PHASE5_START):");
            Serial2.println(gps_longitude, 9);
            Serial2.flush();
            
            previous_Millis = millis();
            phase_state = 5;
            count = 0;
          }

          /*

          current_distance = ultra_distance;
          Serial.println(ultra_distance);
          switch (phase_5)
          {
          case 5:
            current_Millis = millis();
            Serial.print("phase(rotating)=");
            Serial.println(phase_5);
            CanSatLogData.println("-----------------------");
            CanSatLogData.println("ROTATING PHASE");
            CanSatLogData.print("GPS TIME:");
            CanSatLogData.println(gps_time);
            CanSatLogData.println("searching for the goal...");
            CanSatLogData.flush();
            leftturn();
            delay(100);
            slow_rotating();
            if (ultra_distance < 600 && ultra_distance != 0)
            {
              phase_5 = 0;
              stoppage();
              Serial.println("STOP!");
              CanSatLogData.println("-----------------------");
              CanSatLogData.println("STOPPAGE PHASE");
              CanSatLogData.print("GPS TIME:");
              CanSatLogData.println(gps_time);
              CanSatLogData.println("goal is detected!");
              CanSatLogData.flush();
            }          
            break;

          case 0:
            Serial.print("phase(check_former)=");
            Serial.println(phase_5);
            if (abs(current_distance - previous_distance) < EPSILON)
            {
              count++;
            }
            if (count == 5)
            { //イズチェック
              phase_5 = 1;
              time1 = millis();
              distance1 = current_distance;
              count = 0;
              accel();
            }
            previous_distance = current_distance;

            break;

          case 1:
            Serial.print("phase(forward)=");
            Serial.println(phase_5);
            CanSatLogData.println("-----------------------");
            CanSatLogData.println("FORWARD PHASE");
            CanSatLogData.print("GPS TIME:");
            CanSatLogData.println(gps_time);
            CanSatLogData.println("moving for 1000[ms]");
            CanSatLogData.flush();
            time2 = millis();
            forward();
            if (time2 - time1 >= 1000)
            {
              stopping();
              phase_5 = 2;
            }
            break;

          case 2:
            Serial.print("phase(check_later)=");
            Serial.println(phase_5);
            if (abs(current_distance - previous_distance) < EPSILON)
            {
              count++;
            }
            if (count == 5)
            { //イズチェック
              phase_5 = 3;
              distance2 = current_distance;
              count = 0;
            }
            previous_distance = current_distance;
            break;

          case 3:
            Serial.print("phase(judge)=");
            Serial.println(phase_5);
            CanSatLogData.println("-----------------------");
            CanSatLogData.println("JUDGEING PHASE");
            CanSatLogData.print("GPS TIME:");
            CanSatLogData.println(gps_time);
            CanSatLogData.flush();
            if (distance2 - distance1 < 0)
            {
              if (distance2 < 100)
              {
                phase_5 = 4;
                CanSatLogData.println("goal!");
                CanSatLogData.flush();
              }
              else
              {
                phase_5 = 1;
                CanSatLogData.println("approaching!");
                CanSatLogData.flush();
              }
            }
            else
            {
              phase_5 = 5;
              CanSatLogData.println("receding...");
              CanSatLogData.flush();
            }
            break;

          case 4:
            Serial.print("phase(goal)=");
            Serial.println(phase_5);
            Serial.println("GOAL!");
            CanSatLogData.println("-----------------------");
            CanSatLogData.println("STOPPING PHASE");
            CanSatLogData.print("GPS TIME:");
            CanSatLogData.println(gps_time);
            CanSatLogData.println("conglatulations!");
            phase_5 = 6;
            break;

          default:
            break;
          }

          break;
        }
          */




        
        }

        // SDカードへデータを保存する
        Serial2.print(gps_time);
        Serial2.print(",");
        Serial2.print(gps_latitude, 9);
        Serial2.print(",");
        Serial2.print(gps_longitude, 9);
        Serial2.print(",");
        Serial2.print(gps_velocity);
        Serial2.print(",");
        Serial2.print(Temperature);
        Serial2.print(",");
        Serial2.print(Pressure);
        Serial2.print(",");
        Serial2.print(accelX);
        Serial2.print(",");
        Serial2.print(accelY);
        Serial2.print(",");
        Serial2.print(accelZ);
        Serial2.print(",");
        Serial2.print(heading);
        Serial2.print(",");
        Serial2.print(headingDegrees);
        Serial2.print(",");
        Serial2.print(R);
        Serial2.print(",");
        Serial2.print(G);
        Serial2.print(",");
        Serial2.print(B);
        Serial2.print(",");
        Serial2.println(ultra_distance);
        Serial2.flush();
      }
    }
  }
      
    
      
     
}
