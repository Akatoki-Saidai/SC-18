import serial

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=10)

while True:
    rundata = ser.readline().decode("utf-8").rstrip()
    with open("/home/nbus/N-BUS_RunReport.txt", "a") as f:
        f.write(rundata + "\n")
    print(rundata)
