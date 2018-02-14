import cv2
import numpy as np
from operator import itemgetter
from linalg import unit_vector

YELLOW = (0, 255, 255)
RED = (0, 0, 255)
PURPLE = (255, 0, 255)
GREEN = (0, 255, 0)
ORANGE = (100, 100, 255)

def pixel(img_grey, vector):
    if img_grey[vector[1], vector[0]]:
        return 1
    else:
        return 0


def adjust_curve(image, factor=2.5):
    # build a lookup table mapping the pixel values [0, 255] to
    # their steepened curve values
    table = np.array([min(255, i*factor)
                      for i in np.arange(0, 256)]).astype("uint8")

    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)


def find_largest_n_side(img, sides=4):
    """
    Find polygons in color image

    :param img: OpenCV BGR image mat
    :param sides: Number of sides of the wanted polygon
    :return: processed image (grayscale), contour
    """
    # Find edges
    img_grey = cv2.Canny(img, 80, 180)

    # Enlarge edges
    img_grey = cv2.dilate(img_grey, np.ones((3, 3)))

    # Sift trough all contours for the largest rectangle
    img_grey, contours, hierarchy = cv2.findContours(img_grey, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    largest = 0
    result = np.zeros((sides, 2))
    for c in contours:
        approx = cv2.approxPolyDP(c, cv2.arcLength(c, True) * 0.02, True)
        if len(approx) == sides:
            # It has four corners, it must be a rectangle!
            area = cv2.contourArea(approx)
            if area > largest:
                result = approx
                largest = area
    return img_grey, result


def sorted_rect(input_rect, dtype="float32"):
    # -- we'll start
    # by reshaping our contour to be our finals and initializing
    # our output rectangle in top-left, top-right, bottom-right,
    # and bottom-left order
    pts = input_rect.reshape(4, 2)
    rect = np.zeros((4, 2), dtype=dtype)
    # the top-left point has the smallest sum whereas the
    # bottom-right has the largest sum
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    # compute the difference between the points -- the top-right
    # will have the minumum difference and the bottom-left will
    # have the maximum difference
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    return rect


def offset_convex_polygon(polygon, offset, dtype="float32", round_values=True):
    """
    Offsets a convex polygon by a certain amount of pixels
    :param polygon:
    :param offset:
    :return:
    """
    # First offset the playing field to include a bit of the black border
    offset_polygon = np.zeros((4, 2), dtype=dtype)
    for i in range(-1, len(polygon)-1):
        to_prev = polygon[(i - 1)] - polygon[i]
        to_next = polygon[(i + 1)] - polygon[i]

        # the vertex p is moved to p' along the line which halves the angle a between the vectors v1 and v2.
        # The vector w in this direction is
        w = unit_vector(to_prev) + unit_vector(to_next)

        # The new point p' is
        p = polygon[i] + offset * w
        if round_values:
            p = np.around(p)
        offset_polygon[i] = p

    return offset_polygon.astype(dtype)


def rect_from_image_size(width, height):
    return np.array([
        [0, 0],
        [width, 0],
        [width, height],
        [0, height]], dtype="float32")


def find_nested_triangles(img, threshold=150, threshold_type="simple", depth=2):
    triangles = []
    img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if threshold_type == "simple":
        # convert to grayscale and adjust gamma curve
        # img_grey = adjust_curve(img_grey, factor=1.8)
        # Simple adaptive mean thresholding
        values, img_grey = cv2.threshold(img_grey, threshold, 255, cv2.THRESH_BINARY)

    elif threshold_type == "otsu":
        values, img_grey = cv2.threshold(img_grey, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    elif threshold_type == "adaptive":
        img_grey = cv2.adaptiveThreshold(img_grey, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 17, 2)

    elif threshold_type == "canny":
        img_grey = cv2.Canny(img_grey, 50, 150)
        img_grey = cv2.dilate(img_grey, np.ones((2, 2)))
        # img_grey = cv2.dilate(img_grey, np.array([[0,1,0],[1,1,1],[0,1,0]], dtype=np.uint8))

    # Find contours and tree
    img_grey, contours, hierarchy = cv2.findContours(img_grey, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find triangular contours with at least 2 children. These must be our markers!
    for x in range(0, len(contours)):
        k = x
        c = 0

        # Look for nested triangles
        while (hierarchy[0][k][2] != -1):
            # As long as k has children [2], find that child and look for children again.
            # http://docs.opencv.org/3.1.0/d9/d8b/tutorial_py_contours_hierarchy.html
            k = hierarchy[0][k][2]
            c = c + 1

        if hierarchy[0][k][2] != -1:
            c = c + 1

        if c == depth:
            # To do: also check if it runs *exactly* 2 children deep. and not more.
            # This marker has at least two children. Now let's check if it's a triangle.
            approx = cv2.approxPolyDP(contours[x], cv2.arcLength(contours[x], True) * 0.04, True)
            if len(approx) == 3:
                triangles += [approx]

    return img_grey, triangles


def find_largest_rectangle_transform(img, offset, look_for='edges'):

        height, width = np.shape(img)[:2]
        largest_rect = rect_from_image_size(width, height)
        if look_for == 'edges':
            img_grey, largest_rect = find_largest_n_side(img, sides=4)
            # Optionally review edge-finding image
            # img = cv2.cvtColor(img_grey, cv2.COLOR_GRAY2BGR)
        if look_for == '4_blobs':
            img, centers = find_blobs(img, (60, 70, 20), (100, 255, 160), 65)
            if len(centers) > 4:
                largest_rect = np.array(centers[:4])

        # Now that we have our playing field contour, we need to determine
        # the top-left, top-right, bottom-right, and bottom-left
        # points so that we can later warp the image. So we sort the polygon points in this order
        rect = sorted_rect(largest_rect)
        offset_rect = offset_convex_polygon(rect, offset)

        # Present the found rectangles to the user
        cv2.drawContours(img, [rect.astype(int)], -1, GREEN, thickness=8)
        cv2.drawContours(img, [offset_rect.astype(int)], -1, PURPLE, thickness=8)
        cv2.putText(img, "Press y if the playing field is detected, press n if not", (100, 500),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, GREEN, 4)

        # now that we have our rectangle of points, let's compute
        # the width of our new image
        if len(offset_rect) == 4:
            try:
                (tl, tr, br, bl) = offset_rect
                width_a = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
                width_b = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))

                # ...and now for the height of our new image
                height_a = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
                height_b = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))

                # take the maximum of the width and height values to reach
                # our final dimensions
                width = max(int(width_a), int(width_b))
                height = max(int(height_a), int(height_b))

                # construct our destination points which will be used to
                # map the screen to a top-down, "birds eye" view
            except:
                pass
        dst = rect_from_image_size(width - 1, height - 1)

        # calculate the perspective transform matrix and warp
        # the perspective to grab the screen
        M = cv2.getPerspectiveTransform(offset_rect, dst)

        return img, M, dst, width, height


def find_blobs(img, min_hsv, max_hsv, min_size):
    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_blobs = cv2.inRange(img_HSV, min_hsv, max_hsv)
    img_blobs, contours, tree = cv2.findContours(img_blobs, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    blobs = [((0,0), 0)]
    for c in contours:
        M = cv2.moments(c)
        area = M['m00']
        if min_size < area:  # m00 is the area
            cv2.drawContours(img, [c], -1, (255, 0, 0), thickness=3)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            blobs += [([cx, cy], area)]
            cv2.line(img, (cx - 20, cy), (cx + 20, cy), (0, 0, 255), 2)
            cv2.line(img, (cx, cy - 20), (cx, cy + 20), (0, 0, 255), 2)

        blobs = sorted(blobs, key=itemgetter(1), reverse=True) # Sort blobs so the largest are first
    blob_centers = [blob[0] for blob in blobs]

    return img, blob_centers
