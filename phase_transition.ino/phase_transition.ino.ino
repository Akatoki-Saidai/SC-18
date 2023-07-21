void loop()
{
//  if (Serial1.available() > 0) // "Serial1.available() > 0"がないとGPS使えない
//  { 
    char c = Serial1.read();
    gps.encode(c);
//    if (gps.location.isUpdated()) // この中でGPSのデータを使わないと，うまく値が表示されない
    { 
        //現在時刻を保存
        currentMillis = millis();
        
        // センサー値取得
        // for BME280
        Temperature = bme.readTemperature();
        Pressure = bme.readPressure() / 100.0;
        Humid = bme.readHumidity();
        altitude = ( ( pow( 1013.25 /Pressure , 1/5.257) - 1 ) * ( Temperature + 273.15 ) ) / 0.0065;

        // for MPU6050
        mySensor.accelUpdate();
        if (mySensor.accelUpdate() == 0)
        {
          accelX = accelermetor.X();
          accelY = accelermetor.Y();
          accelZ = accelermetor.Z();
          accelSqrt = mySensor.accelSqrt();
          if(fabs(accelZ)>4.0){
            ESP.restart();
          }
        }

        // for GPS
        gps_latitude = gps.location.lat();
        gps_longitude = gps.location.lng();       
        gps_time = gps.time.value();
        gps_velocity = gps.speed.mps();
        
        // for HY-SRF05
        digitalWrite(trigPin, LOW); 
        delay(2); 
        digitalWrite( trigPin, HIGH ); // 超音波を10ms間送信
        delay(10);
        digitalWrite( trigPin, LOW ); // 超音波を停止      
        Duration = pulseIn( echoPin, HIGH ); // センサからの入力
        if (Duration > 0) {
          Duration = Duration/2; // 往復距離を半分にする
          ultra_distance = Duration*(331.5+0.6*Temperature)*100/1000000; // 音速を340m/sに設定
        } 

          //########## 遠距離フェーズ ##########
          case 4:
            if(phase_state != 4)
            {
              Serial.println("Phase4: transition completed");
              // LogDataの保存
              CanSatLogData.println(gps_time);
              CanSatLogData.println("Phase4: transition completed");
              CanSatLogData.flush();

              
 
              // BNOの初期化
              while (!compass.begin())
              {
                Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
                delay(500);
              }
              if (compass.isHMC())
              {
                Serial.println("Initialize HMC5883");
                compass.setRange(HMC5883L_RANGE_1_3GA);
                compass.setMeasurementMode(HMC5883L_CONTINOUS);
                compass.setDataRate(HMC5883L_DATARATE_15HZ);
                compass.setSamples(HMC5883L_SAMPLES_8);
              }
              else if (compass.isQMC())
              {
                Serial.println("Initialize QMC5883");
                compass.setRange(QMC5883_RANGE_2GA);
                compass.setMeasurementMode(QMC5883_CONTINOUS);
                compass.setDataRate(QMC5883_DATARATE_50HZ);
                compass.setSamples(QMC5883_SAMPLES_8);
              }
              // BNOの初期化終了

              // パラシュートと絡まらないように約3秒間前進
              accel();
              forward();
              delay(1000);
              brake();
              off();
              
              
              // BNOのキャリブレーション
              delay(100);
              Serial.println("calibration rotating!");
              while (CalibrationCounter < 551)
              {
                Vector norm = compass.readNormalize();
                rotating();
                if (CalibrationCounter == 550)
                {
                  off();
                  Serial.println("calibration stopping!");
                  delay(2000);
                  CalibrationCounter = CalibrationCounter + 1;
                }
                else
                {
                  CalibrationCounter = CalibrationCounter + 1;
                  Serial.print("CalibrationCounter = ");
                  Serial.println(CalibrationCounter);
                }
              }
              // BNOのキャリブレーション終了
              
              phase_state = 4;
            }
            
            CurrentDistance = CalculateDis(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude); // 現在位置とゴールとの距離を計算

            if (desiredDistance >= CurrentDistance)
            {
              // カラーコーンとの距離が理想値よりも小さい場合は次のフェーズに移行する
              phase = 5;
            }else{
              delay(100);
              accel();
              forward();
              delay(500);
              brake();
              off();
              // Goalまでの偏角を計算する
              Angle_Goal = CalculateAngle(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
  
              Sum_headingDegrees = 0.0;
              for (int i = 0; i < 15; i++){
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
              Angle_gy271 = Sum_headingDegrees / 15;

              // どちらに回ればいいか計算
              rrAngle = -Angle_gy271 + Angle_Goal;
              if (rrAngle < 0){
                rrAngle += 360;
              }
              if (rrAngle > 360){
                rrAngle -= 360;
              }
              llAngle = Angle_gy271 - Angle_Goal;
              if (llAngle < 0){
                llAngle += 360;
              }
              if (llAngle > 360){
                llAngle -= 360;
              }
  
              if (rrAngle > llAngle){
                //反時計回り
                if (llAngle > 20){
                  rotating();
                  delay(150);
                  off();
                }
              }else{
                //時計回り
                if (rrAngle > 20){
                  reverse_rotating();
                  delay(150);
                  off();
                }
              }
            }

            break;

          //########## 中距離フェーズ ##########
          case 5:
            if(phase_state != 5)
            {
              Serial.println("Phase5: transition completed");
              // LogDataの保存
              CanSatLogData.println(gps_time);
              CanSatLogData.println("Phase5: transition completed");
              CanSatLogData.flush();

              phase_state = 5;
            }
            
            nowmillis = millis(); //現在時刻をnowmillisに保存

            //初期化
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
  
            CurrentDistance = CalculateDis(GOAL_lng, GOAL_lat, gps_longitude, gps_latitude);
            
            if (1.3 >= CurrentDistance || (ultra_distance < 300 && ultra_distance != 0)){
              // カラーコーンとの距離が理想値よりも小さい場合は次のフェーズに移行する
              phase = 6;
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

              // ゴール方向を向くまで回転
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
              
              //回転停止
              off();
  
              //少し進む
              delay(100);
              accel();
              forward();
              delay(500);
              brake();
              off();
            }
            break;

//} // loop関数の閉じ