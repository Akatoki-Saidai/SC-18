#include <Wire.h>                                                         //BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
float temp;
float pressure;
float humid;

#include <Wire.h>　　　　　　　　　　　　　　　　　　　　　　　　　　　　//BNO055
#include <Adafruit_BNO055.h>
#include <Ticker.h>
Ticker bno055ticker; //タイマー割り込み用のインスタンス
#define BNO055interval 10 //何ms間隔でデータを取得するか
//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); //ICSの名前, デフォルトアドレス, 謎
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


#include <TinyGPS++.h>                                                     //GPS
TinyGPSPlus gps;

int cutparac = 23;          //切り離し用トランジスタのピン番号の宣言           //nicromewire
int outputcutsecond = 3;    //切り離し時の9V電圧を流す時間，単位はsecond

// 前進                            //motor
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
}




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


    delay(10000);                                                               //nicromeewire
   Serial.print("WARNING: 9v voltage on.\n");
   digitalWrite(cutparac, HIGH); //オン
   delay(outputcutsecond*1000);//十秒間電流を流す
   Serial.print("WARNING: 9v voltage off.\n");
   digitalWrite(cutparac, LOW); //オフ
      delay(5000);


    pinMode(21, INPUT_PULLUP); //SDA 21番ピンのプルアップ(念のため)              //BNO055
  pinMode(22, INPUT_PULLUP); //SDA 22番ピンのプルアップ(念のため)

  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  if (!bno.begin()) // センサの初期化
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(10000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(false);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  bno055ticker.attach_ms(BNO055interval, get_bno055_data);
}


    
}

void loop(){
  forward();                                   //moter
  
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

  void get_bno055_data(void)                                   //BNO055
{
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
  
  /*
  // ジャイロセンサ値の取得と表示
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print(" 　Gy_xyz:");
  Serial.print(gyroscope.x());
  Serial.print(", ");
  Serial.print(gyroscope.y());
  Serial.print(", ");
  Serial.print(gyroscope.z());
  */
  

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

  //GPS
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {
      Serial.print("LAT:  "); Serial.println(gps.location.lat(), 9);
      Serial.print("LONG: "); Serial.println(gps.location.lng(), 9);
    }
  }
  



    
    
  
}
