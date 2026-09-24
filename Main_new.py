import cv2
import mediapipe
import serial
from math import sqrt, atan2, pi, sin, cos, radians, asin, acos

#settings
adjust_factors: dict = {'Hand angle': (-5, 1),
                        'Hand distance': (0, 1),
                        'Hand angle yz': (-60, 5)}
framerate: int = 200
dampen_strength: dict = {'general': 5,
                         0: 0,
                         1: 0,
                         2: 0,
                         3: 0}
#higher values can cause delayed responses. Delay(ms) = (framerate//1000)*dampen_strength

#constants
arduino_port = 'COM13'
video_port = 0
image_flip = True
flipCode = 1

try:
    arduino = serial.Serial(arduino_port, 9600)
except serial.serialutil.SerialException:
    print("Serial port was unable to open")
    while True:
        input_value = input("try another port? (port/n): ")
        if input_value == "n":
            break
        try:
            arduino = serial.Serial(input_value, 9600)
        except serial.serialutil.SerialException:
            print("Serial port was unable to open")
        else:
            break

webcam = cv2.VideoCapture(video_port)
myHands = mediapipe.solutions.hands.Hands()
drawingUtils = mediapipe.solutions.drawing_utils

#for function DistToAngles
distanceValues = {'Arm Length': 8.5,
                  'Max Length': 14}

#variables
x1 = y1 = x2 = y2 = 0
dx1 = dx2 = dy1 = dy2 = dz1 = dz2 = 0
hand_angle: float = 0
hand_distance: float = 0
hand_angle_yz: float = 0

final_data: list = [90, 45, 52, 0]

#Initialize previous values for function dampen
prev_values: list = []

for _ in range(len(final_data)):
    prev_values.append([])

for i in dampen_strength:
    if i == 'general': continue
    for _ in range(dampen_strength['general'] if dampen_strength[i] == 0 else dampen_strength[i]):
        prev_values[i].append(final_data[i])

print(prev_values)

servoAngle: int = 0

lenTTM: float = 0 #normalised distance from middle finger to center point
lenBTM: float = 0 #normalised distance from wrist to center point

def clamp(value, min_val, max_val):
    if value < min_val:
        return min_val
    elif value > max_val:
        return max_val
    else:
        return value

def sendData(data: list, device: serial.Serial):
    while len(data) < 4:
        data.append(0)

    final_data = ""
    for val in data:
        val = int(clamp(val, 0, 180))
        final_data += "0"*(3-len(str(val))) + str(val)

    device.write(final_data.encode())

def myMap(value: float, min1: float, max1: float, min2: float, max2: float):
    range1 = max1 - min1
    range2 = max2 - min2
    value_normalized = (value - min1) / range1
    new_value = value_normalized * range2 + min2
    return new_value

def distToAngles(distance: float, angle_yz: float):
    arm = distanceValues['Arm Length']
    total = distanceValues['Max Length']
    distance = distance * distanceValues['Max Length']
    #print(distance, angle_yz)
    alpha = acos(clamp((arm**2 + (total-arm)**2 - distance**2)/(2*arm*(total-arm)), -1, 1))
    alpha = (alpha * 180/pi)
    angle1 = angle_yz - alpha + 90 #convert the angles to servo coordinate system, servo 0 is at 90 degrees
    angle2 = alpha
    return angle1, angle2

def dampen(x: list, previous_values: list):
    new_values = []

    #append new values of x while removing the oldest value
    for i in range(len(x)):
        previous_values[i].append(x[i])
        previous_values[i].pop(0)

    #take the average of the values
    for i in range(len(x)):
        new_values.append(sum(previous_values[i])//(dampen_strength['general'] if dampen_strength[i] == 0 else dampen_strength[i]))

    #return new values and updated previous values
    return new_values, previous_values


while True:
    _, image = webcam.read()
    if image_flip:
        image = cv2.flip(image, flipCode)
    key = cv2.waitKey(1000//framerate)
    final_data = [90,45,52,0] #reset data list

    #draw landmarks on the image
    imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    output = myHands.process(imageRGB)
    hands = output.multi_hand_landmarks

    if hands:
        for point in hands:
            drawingUtils.draw_landmarks(image, point, connections=[(9, 12)])
            landmarks = point.landmark
            for i, lm in enumerate(landmarks):
                #get the position of the landmark
                x = int(lm.x * image.shape[1])
                y = int(lm.y * image.shape[0])

                #8 is the index finger
                if i == 8:
                    cv2.circle(img = image, center = (x,y), radius = 10, color = (0, 0, 255), thickness = 2)
                    x1 = x
                    y1 = y
                #4 is the thumb
                if i == 4:
                    cv2.circle(img = image, center = (x,y), radius = 10, color = (0, 0, 255), thickness = 2)
                    x2 = x
                    y2 = y

            #COORDINATE SYSTEM:
            #x: left-right
            #y: up-down
            #z: forward-backward (toward/away from camera)

            # get xy angle of the hand
            dx1, dy1 = landmarks[9].x, landmarks[9].y
            dx2, dy2 = landmarks[12].x, landmarks[12].y
            dx = dx2 - dx1
            dy = dy2 - dy1
            hand_angle = atan2(dy, dx)
            #range: -pi to 0
            hand_angle = clamp(hand_angle, -pi, 0)
            hand_angle = -(hand_angle/pi)*180
            hand_angle += adjust_factors['Hand angle'][0]

            #get xz angle of the hand
            dx1, dy1, dz1 = landmarks[9].x, landmarks[9].y, landmarks[9].z
            dx2, dy2, dz2 = landmarks[0].x, landmarks[0].y, landmarks[0].z

            #find length of hand relative to 2 axis, x and y
            hand_relative_len = sqrt((dx2-dx1)**2 + (dy2-dy1)**2)

            #direct mapping of distance to angle
            hand_relative_len = clamp(hand_relative_len, 0, 0.3)
            hand_angle_yz = myMap(hand_relative_len, 0, 0.3, 0, 45)
            hand_angle_yz = clamp(hand_angle_yz, 0, 45)
            print(hand_angle_yz)

            lenBTM = landmarks[0].y - landmarks[9].y
            lenTTM = 1-(landmarks[9].y - landmarks[12].y)

            #get normalized distance of the hand
            hand_distance = myMap(landmarks[9].y, lenTTM, lenBTM, 0, 1)

    #draw a line between x1, y1 and x2, y2
    cv2.line(image, (x1, y1), (x2, y2), color = (0, 255, 0), thickness = 2)

    #draw image
    cv2.imshow("Volume control", image)

    #calculate the distance between the lines
    dist = sqrt((x2 - x1)**2 + (y2 - y1)**2)//4

    #close window when esc key is pressed
    if key == 27:
        break

    servoAngle = int(clamp(hand_angle, 0, 180))

    final_data[0] = servoAngle
    final_data[1], final_data[2] = distToAngles(hand_distance, hand_angle_yz)

    final_data, prev_values = dampen(final_data, prev_values)

    print(final_data)

    sendData(data=final_data, device=arduino)

webcam.release()
cv2.destroyWindow("Volume control")