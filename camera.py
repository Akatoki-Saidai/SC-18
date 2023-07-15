import cv2
import numpy as np
from picamera2 import Picamera2


def red_detect(frame):
    # HSV色空間に変換
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 赤色のHSVの値域1
    hsv_min = np.array([0, 160, 100])
    hsv_max = np.array([25, 255, 255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    # 赤色のHSVの値域2
    hsv_min = np.array([330, 153, 230])
    hsv_max = np.array([360, 255, 255])
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

    # マスクされた画像を表示
    # 結果表示
    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    # キー入力を取得
    key = cv2.waitKey(25)

    # qキーを押すと終了(手動停止)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

# カメラを終了
picam2.release()

# ウィンドウを閉じる
cv2.destroyAllWindows()

