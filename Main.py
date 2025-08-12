import cv2
import mediapipe
import pyautogui
from math import sqrt

webcam = cv2.VideoCapture(1)
myHands = mediapipe.solutions.hands.Hands()
drawingUtils = mediapipe.solutions.drawing_utils
x1 = y1 = x2 = y2 = 0
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
    #draw a line between x1, y1 and x2, y2
    cv2.line(image, (x1, y1), (x2, y2), color = (0, 255, 0), thickness = 2)

    #draw image
    cv2.imshow("Volume control", image)

    #calculate the distance between the lines
    dist = sqrt((x2 - x1)**2 + (y2 - y1)**2)//4

    #control volume based on dist
    if dist > 50 and hands:
        pyautogui.press("volumeup")
    elif hands:
        pyautogui.press("volumedown")

    #close window when esc key is pressed
    if key == 27:
        break

webcam.release()
cv2.destroyWindow("Volume control")
