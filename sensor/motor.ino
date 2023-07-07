void setup() {
  Serial.begin(115200);
  ledcSetup(0, 490, 8);
  ledcSetup(1, 490, 8);
  ledcSetup(2, 490, 8);
  ledcSetup(3, 490, 8);

  ledcAttachPin(32, 0);
  ledcAttachPin(33, 1);
  ledcAttachPin(26, 2);
  ledcAttachPin(25, 3);
/*
  int CalibrationCounter = 1;

  Serial2.println("calibration rotationg...");
  while(CalibrationCounter < 551){
    rotating();
    if(CalibrationCounter = 550){
      stoppage();
      Serial2.println("calibration stopping");
      delay(2000);
      CalibrationCounter += 1;
    }
    else{
      CalibrationCounter += 1;
      Serial2.println("calibrationCounter = ");
      Serial2.println(CalibrationCounter);
    }
  }
  */
  
}

void loop() {

  //stoppage();
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
}
