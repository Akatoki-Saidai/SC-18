import cv2
import numpy as np
from picamera2 import Picamera2
import time
import serial


#シリアルポート開放
ser = serial.Serial('/dev/ttyAMA0', 19200, timeout=10)


#赤色検知関数の定義
def red_detect(frame):
    # HSV色空間に変換
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1
    hsv_min = np.array([0, 117, 70])
    hsv_max = np.array([12, 255, 255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    # 赤色のHSVの値域2
    hsv_min = np.array([168, 117, 70])
    hsv_max = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

    return mask1 + mask2

def camera_rest():
    camera_sleep += 1
    if camera_sleep == 400:
        picam2.close()
        time.sleep(10)
        picam2 = Picamera2()
        picam2.configure(config)
        picam2.start()
        #print("camera reset was done ")
        camera_sleep = 0

def track_red():
    if frame_center_x -  50 <= center_x <= frame_center_x + 50:
        #print("赤色物体は画像の中心にあります。")#直進
        camera_order = 1
    elif center_x > frame_center_x + 50:
        #print("赤色物体は画像の右側にあります。")#右へ
        camera_order = 2
    elif center_x < frame_center_x - 50:
        #print("赤色物体は画像の左側にあります。")#左へ
        camera_order = 3
            


#カメラの設定
picam2 = Picamera2()
config = picam2.create_preview_configuration({"format": 'XRGB8888', "size": (480, 360)})
#config["main"]
picam2.configure(config)

#カメラの詳細設定(一応設定)
picam2.brightness = 40 #輝度(0～100)
picam2.saturation = 30   #彩度(-100～100)
picam2.ISO = 0 #ISO感度(0～1600, 0は自動)

picam2.shutter_speed = 100000 #シャッター速度(マイクロ秒,0は自動)
picam2.framerate = 10
picam2.exposure_compensation = -20 #露出補正(-25～25)
picam2.exposure_mode = 'auto' #露出モード 
picam2.meter_mode = 'auto' #測光モード
picam2.awb_mode = 'auto' #ホワイトバランス
picam2.awb_gains = (1.0,1.0) #手動AWB調整(0.0～8.0)
picam2.image_effect = 'none' #画像効果
picam2.color_effects = None #カラー効果

picam2.rotation = 0 #回転(0～359）
picam2.hflip = False #水平反転
picam2.vflip = True #垂直反転
picam2.crop = (0.0, 0.0, 1.0, 1.0) #切り抜き(0.0～1.0)
camera_sleep = 0


#カメラの起動用の変数の初期化
CameraStart = 0

#シリアル通信(制御履歴)スタート
while True:
    rundata = ser.readline().decode("shift-jis").rstrip() #デコードの文字コードが正しいかに注意
    with open("/boot/N-BUS_RunReport.txt", "a") as f:
        f.write(rundata + "\n")
        f.flush()
    print(rundata)


    #ESPから近距離フェーズへの移行要請を受信
    if rundata == "CameraStart":
        CameraStart = 1

    #近距離フェーズであれば画像認識を開始
    if CameraStart == 1:
        # カメラのキャプチャ
        #cap = cv2.VideoCapture(0)
        picam2.start()

        #カメラの休止(熱対策)
        camera_rest()
        

        # フレームを取得
        #ret, frame = cap.read()
        frame = picam2.capture_array()
    
        # 赤色を検出
        mask = red_detect(frame)


        # 画像の中にある領域を検出する
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
        #画像の中に赤の領域があるときに赤色を処理
        if 0 < len(contours):
            
            # 輪郭群の中の最大の輪郭を取得する-
            biggest_contour = max(contours, key=cv2.contourArea)

            # 最大の領域の外接矩形を取得する
            rect = cv2.boundingRect(biggest_contour)
  
            #最大の領域の中心座標を取得する
            center_x = (rect[0] + rect[2] // 2)
            center_y = (rect[1] + rect[3] // 2)

            # 最大の領域の面積を取得する-
            area = cv2.contourArea(biggest_contour)

            # 最大の領域の長方形を表示する
            cv2.rectangle(frame, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 0, 255), 2)

            # 最大の領域の中心座標を表示する
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

            # 最大の領域の面積を表示する
            cv2.putText(frame, str(area), (rect[0], rect[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)

            # 画像の中心座標
            frame_center_x = frame.shape[1] // 2

        #画像の中に赤色物体がない(ゆっくり時計回りして探す)
        else:
            camera_order = 4

            #画像の赤色物体の結果からモーターを動かす
            #中心座標のx座標が画像の中心より大きいか小さいか判定
            if area > 90000:
                #print("十分近い")
                camera_order = 0 #ゴールしたと判定
                break
            elif area < 20:
                camera_order = 4 #標的が小さすぎる(ゆっくり時計回りして探す)
            else:
                track_red()
        

        #モーターの動作をESPに送信
        ser.write(camera_order.encode())


        # 面積のもっとも大きい領域を表示
        # 結果表示
        cv2.imshow("Frame", frame)
        #cv2.imshow("Mask", mask)
    
    
    # qキーを押すと終了(手動停止)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

#終了
#シリアルポートを閉じる
ser.close()
#カメラを終了
picam2.close()
#ウィンドウを閉じる
cv2.destroyAllWindows()
