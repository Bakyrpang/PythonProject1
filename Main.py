import cv2
import mediapipe
import serial
from math import sqrt, atan2, pi, sin, cos, radians, asin

#settings
adjust_factors = {'Hand angle': -5,
                  'Hand distance': (0, 10),
                  'Hand angle yz': (-90)}

#constants
arduino = serial.Serial('COM4', 9600)
webcam = cv2.VideoCapture(1)
myHands = mediapipe.solutions.hands.Hands()
drawingUtils = mediapipe.solutions.drawing_utils

#for function DistToAngles
distanceValues = {'Arm Length': 8,
                  'Max Length': 15}

#variables
x1 = y1 = x2 = y2 = 0
dx1 = dx2 = dy1 = dy2 = dz1 = dz2 = 0
hand_angle: float = 0
hand_distance: float = 0
hand_angle_yz: float = 0

final_data: list = [0, 0, 0, 0]

servoAngle: int = 0

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
    angle = 90-(asin(clamp(distance, 0, 1)*total/(2*arm)))*180/pi
    angle = clamp(angle, 0, 180)
    angle1 = int(myMap(angle, 0, 180, 45, 140))
    angle2 = myMap(angle, 0, 180, 52, 160)
    angle2 = clamp(angle2 + angle_yz, 52, 160)
    angle2 = int(angle2)
    return angle1, angle2

while True:
    _, image = webcam.read()
    key = cv2.waitKey(17)
    final_data = [90,45,52,0] #reset data list

    #draw landmarks on the image
    imageRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    output = myHands.process(imageRGB)
    hands = output.multi_hand_landmarks

    if hands:
        for point in hands:
            drawingUtils.draw_landmarks(image, point)
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

            # get xy angle of the hand
            dx1, dy1 = landmarks[0].x, landmarks[0].y
            dx2, dy2 = landmarks[9].x, landmarks[9].y
            dx = dx2 - dx1
            dy = dy2 - dy1
            hand_angle = atan2(dy, dx)
            #range: -pi to 0
            hand_angle = clamp(hand_angle, -pi, 0)
            hand_angle = -(hand_angle/pi)*180
            hand_angle += adjust_factors['Hand angle']

            #get xz angle of the hand
            dz1, dy1 = landmarks[0].z, landmarks[0].y
            dz2, dy2 = landmarks[9].z, landmarks[9].y
            dz = dz2 - dz1
            dxy = cos(radians(hand_angle))*dx + sin(radians(hand_angle))*dy
            hand_angle_yz = atan2(dxy, dz)
            hand_angle_yz = -(hand_angle_yz/pi)*180
            hand_angle_yz += adjust_factors['Hand angle yz']
            hand_angle_yz = clamp(hand_angle_yz, 0, 45)

            #get normalized distance of the hand
            hand_distance = landmarks[9].y

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

    print(final_data)

    sendData(data=final_data, device=arduino)

    previousServoAngle = servoAngle

webcam.release()
cv2.destroyWindow("Volume control")
