import serial

arduino = serial.Serial('COM3', 9600)

while True:
    l = ""
    for i in range(3):
        x = input(f"Enter value {i+1}: ")
        if len(x) == 1:
            x = "00"+x
        elif len(x) == 2:
            x = "0"+x
        l += str(255-int(x))
    arduino.write(l.encode())
