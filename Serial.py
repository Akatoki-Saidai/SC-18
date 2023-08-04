import serial

ser = serial.Serial('/dev/ttyAMA0', 19200, timeout=10)

if True:
    camera_order = "Hello, ESP32" #ここに1~4を入れてESPに送信
    byte_camera_order = camera_order.encode()

while True:
    ser.write(byte_camera_order)
    rundata = ser.readline().decode("utf-8").rstrip()
    with open("/home/nbus/N-BUS_RunReport.txt", "a") as f:
        f.write(rundata)
    print(rundata)
ser.close()
