import serial

ser = serial.Serial('/dev/ttyAMA0', 19200, timeout=10)

if True:
    camera_order = "2" #ここに1~4を入れてESPに送信
    
while True:
    ser.write(camera_order.encode())
    rundata = ser.readline().decode("shift-jis").rstrip()
    with open("/boot/N-BUS_RunReport.txt", "a") as f:
        f.write(rundata + "\n")
        f.flush()
    print(rundata)
ser.close()
