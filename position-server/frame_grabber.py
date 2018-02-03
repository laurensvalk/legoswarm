import cv2, time
from settings import server_settings

# Initialize output window
cv2.namedWindow("cam",cv2.WINDOW_OPENGL)
cap = cv2.VideoCapture(0)
cap.set(3, server_settings['WIDTH'])
cap.set(4, server_settings['HEIGHT'])

while True:
    ok, img = cap.read()
    if not ok:
        continue    #and try again.

    cv2.imshow("cam", img)

    keypress = cv2.waitKey(100)

    if keypress == ord('q'):
        break
    elif keypress == ord(' '):
        cv2.imwrite("test_images/{0}.jpg".format(int(time.time())), img)
