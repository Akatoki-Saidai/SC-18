import cv2
import numpy as np
from picamera2 import Picamera2


def red_detect(frame):
    # HSV色空間に変換
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1
    hsv_min = np.array([0, 117, 70])
    hsv_max = np.array([15, 255, 255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    # 赤色のHSVの値域2
    hsv_min = np.array([167, 117, 70])
    hsv_max = np.array([179, 255, 255])
    mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

    return mask1 + mask2


#カメラの設定
picam2 = Picamera2()
config = picam2.create_preview_configuration({"format": 'XRGB8888', "size": (640, 480)})
#config["main"]
picam2.configure(config)

# カメラのキャプチャ
#cap = cv2.VideoCapture(0)
picam2.start()

while True:
    # フレームを取得
    #ret, frame = cap.read()
    frame = picam2.capture_array()
    
    # 赤色を検出
    mask = red_detect(frame)


    # 画像の中にある領域を検出する
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    #画像の中に赤の領域があるときにループ
    if 0 < len(contours):
        
            # 輪郭群の中の最大の輪郭を取得する-
            biggest_contour = max(contours, key=cv2.contourArea)

            # 最大の領域の外接矩形を取得する
            rect = cv2.boundingRect(biggest_contour)
  
            #最大の領域の中心座標を取得する
            center = (rect[0] + rect[2] // 2, rect[1] + rect[3] // 2)

            # 最大の領域の面積を取得する-
            area = cv2.contourArea(biggest_contour)

            # 最大の領域の長方形を表示する
            cv2.rectangle(frame, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 0, 255), 2)

            # 最大の領域の中心座標を表示する
            cv2.circle(frame, center, 5, (0, 255, 0), -1)

            # 最大の領域の面積を表示する
            cv2.putText(frame, str(area), (rect[0], rect[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)



    # 面積のもっとも大きい領域を表示
    # 結果表示
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    # qキーを押すと終了(手動停止)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

# カメラを終了
picam2.release()

# ウィンドウを閉じる
cv2.destroyAllWindows()

