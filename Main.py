import cv2
import mediapipe
import serial
from math import sqrt, atan2, pi

#settings
adjust_factors = {'Hand angle': -5}

arduino = serial.Serial('COM4', 9600)

webcam = cv2.VideoCapture(1)
myHands = mediapipe.solutions.hands.Hands()
drawingUtils = mediapipe.solutions.drawing_utils

x1 = y1 = x2 = y2 = 0
dx1 = dx2 = dy1 = dy2 = 0
hand_angle: float = 0

servoAngle: int = 0
previousServoAngle: int = 0

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
        val = clamp(val, 0, 180)
        final_data += "0"*(3-len(str(val))) + str(val)

    print(final_data)
    device.write(final_data.encode())

while True:
    _, image = webcam.read()
    key = cv2.waitKey(17)

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

            # get angle of the hand
            dx1, dy1 = landmarks[0].x, landmarks[0].y
            dx2, dy2 = landmarks[9].x, landmarks[9].y
            dx = dx2 - dx1
            dy = dy2 - dy1

            hand_angle = atan2(dy, dx)
            #range: -pi to 0
            hand_angle = clamp(hand_angle, -pi, 0)
            hand_angle = -(hand_angle/pi)*180
            hand_angle += adjust_factors['Hand angle']

    #draw a line between x1, y1 and x2, y2
    cv2.line(image, (x1, y1), (x2, y2), color = (0, 255, 0), thickness = 2)

    #draw image
    cv2.imshow("Volume control", image)

    #calculate the distance between the lines
    dist = sqrt((x2 - x1)**2 + (y2 - y1)**2)//4

    #control volume based on dist
    if dist > 50 and hands:
        servoAngle += 5
    elif hands:
        servoAngle -= 5

    #close window when esc key is pressed
    if key == 27:
        break

    servoAngle = clamp(servoAngle, 0, 180)

    sendData(data=[int(hand_angle)], device=arduino)

    previousServoAngle = servoAngle

webcam.release()
cv2.destroyWindow("Volume control")
