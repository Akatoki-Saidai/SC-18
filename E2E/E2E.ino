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


//CDcard
#include <SPI.h> // SDカードはSPI通信
#include <SD.h>
File CanSatLogData;
File SensorData;
const int sck = 13, miso = 15, mosi = 14, ss = 27;

//GPS
#include <TinyGPS++.h>
#inlcude<HardwareSerial.h>
TinyGPSlus gps;
HardwareSerial SerialGPS(1);
double GOAL_lat = ;
double GOAL_lng = ;

//nicromewire
int cutparac = 23;
int outputcutsecond = 3;

//motor
//forward
void forward(){
  ledWrite(0,0);
  ledWrite(1,10000);
  ledWrite(2,0);
  ledWrite(3,10000);
}
//stop
void stoppage(){
  ledWrite(0,0);
  ledWrite(1,0);
  ledWrite(2,0);
  ledWrite(3,0);
}
//rotate
void rotating(){
  ledWrite(0,0);
  ledWrite(1,5000);
  ledWrite(2,5000);
  ledWrite(3,0);
}
//reverse_rotate
void reverse_rotating(){
  ledWrite(0,5000);
  ledWrite(1,0);
  ledWrite(2,0);
  ledWrite(3,5000);
}


void setup(){
  Serial.begin(115200);

//BME280
  bool status = bme.begin(0x76);
  if(!status) {
    Serial.println("BME280 sensor can't be used);
    while(1);
  }

// SD Card initialization
  SPI.begin(sck, miso, mosi, ss);
  SD.begin(ss, SPI);
  CanSatLogData = SD.open("/CanSatLogData.txt", FILE_APPEND);
  SensorData = SD.open("/SensorData.csv", FILE_APPEND);
  CanSatLogData.println("START_RECORD");
  CanSatLogData.flush();
  SensorData.println("gps_time,gps_latitude,gps_longitude,gps_velocity,Temperature,Pressure,Humid,accelX,accelY,accelZ,Angle_gy271,ultra_distance");
  SensorData.flush();

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

/*
//nicromewire
  pinMode(cutparac, OUTPUT);      //切り離し用トランジスタの出力宣言
  digitalWrite(cutparac, LOW);    //切り離し用トランジスタの出力オフ

  delay(10000);
  Serial.print("WARNING: 9v voltage on.\n");
  digitalWrite(cutparac, HIGH); //オン
  delay(outputcutsecond*1000);//十秒間電流を流す
  Serial.print("WARNING: 9v voltage off.\n");
  digitalWrite(cutparac, LOW); //オフ
  delay(5000);
//nicromewire
*/

//BNO055
  Wire.begin(21,22);
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

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


  //motor
  forward();
  //motor


//BME280
  temp=bme.readTemperature();
  pressure=bme.readPressure() / 100.0F;
  humid=bme.readHumidity();
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print("温度 ;");
  Serial.print(temp);
  Serial.println(" °C");
  Serial.print("高度:");
  Serial.print(altitude-30);
  Serial.println("m");
   
  Serial.print("気圧 ;");
  Serial.print(pressure);
  Serial.println(" hPa");
  Serial.print("湿度 ;");
  Serial.print(humid);
  Serial.println(" %");
  delay(1000);
//BME280

  
  

//BNO055
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
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
//BNO055
  
  
//GPS
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {
      Serial.print("LAT:  "); Serial.println(gps.location.lat(), 9);
      Serial.print("LONG: "); Serial.println(gps.location.lng(), 9);
    }
  }
//GPS
     
}
