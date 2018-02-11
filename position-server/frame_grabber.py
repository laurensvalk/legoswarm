import cv2, time
from antoncv import find_blobs
from settings import server_settings

# Initialize output window
cv2.namedWindow("cam")
cap = cv2.VideoCapture(0)
cap.set(3, server_settings['WIDTH'])
cap.set(4, server_settings['HEIGHT'])
# cap.set(3, 1280)
# cap.set(4, 720)


def nothing(x):
    pass

hmin = 30
hmax = 68

smin = 70
smax = 255

vmin = 20
vmax = 160

min_size=65
max_size=130

cv2.createTrackbar('H min','cam',hmin,360,nothing)
cv2.createTrackbar('H max','cam',hmax,360,nothing)

cv2.createTrackbar('S min','cam',smin,255,nothing)
cv2.createTrackbar('S max','cam',smax,255,nothing)

cv2.createTrackbar('V min','cam',vmin,255,nothing)
cv2.createTrackbar('V max','cam',vmax,255,nothing)

cv2.createTrackbar('min size','cam',min_size, 1000,nothing)
# cv2.createTrackbar('max size','cam',max_size, 10000,nothing)


while True:
    ok, img = cap.read()
    if not ok:
        continue    #and try again.

    img, blob_centers = find_blobs(img, (hmin, smin, vmin), (hmax, smax, vmax), min_size)

    h,w = img.shape[:2]

    scaled = cv2.resize(img, (w//3,h//3))
    cv2.imshow("cam", scaled)

    keypress = cv2.waitKey(100)
    print((hmin, smin, vmin), (hmax, smax, vmax), min_size, max_size)

    hmin = cv2.getTrackbarPos('H min', 'cam')
    hmax = cv2.getTrackbarPos('H max', 'cam')
    smin = cv2.getTrackbarPos('S min', 'cam')
    smax = cv2.getTrackbarPos('S max', 'cam')
    vmin = cv2.getTrackbarPos('V min', 'cam')
    vmax = cv2.getTrackbarPos('V max', 'cam')
    min_size = cv2.getTrackbarPos('min size', 'cam')
    # max_size = cv2.getTrackbarPos('max size', 'cam')
    if keypress == ord('q'):
        break
    elif keypress == ord(' '):
        cv2.imwrite("test_images/{0}.jpg".format(int(time.time())), img)
