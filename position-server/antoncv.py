import cv2
import numpy as np

from linalg import unit_vector


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


def find_nested_triangles(img, threshold=150, threshold_type="simple"):
    triangles = []
    img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if threshold_type == "simple":
        # convert to grayscale and adjust gamma curve
        img_grey = adjust_curve(img_grey, factor=1.8)
        # Simple adaptive mean thresholding
        values, img_grey = cv2.threshold(img_grey, threshold, 255, cv2.THRESH_BINARY)

    elif threshold_type == "otsu":
        values, img_grey = cv2.threshold(img_grey, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    elif threshold_type == "adaptive":
        img_grey = cv2.adaptiveThreshold(img_grey, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 17, 2)

    elif threshold_type == "canny":
        img_grey = cv2.Canny(img_grey, 50, 150)
        img_grey = cv2.dilate(img_grey, np.ones((3, 3)))

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

        if c == 2:
            # To do: also check if it runs *exactly* 2 children deep. and not more.
            # This marker has at least two children. Now let's check if it's a triangle.
            approx = cv2.approxPolyDP(contours[x], cv2.arcLength(contours[x], True) * 0.05, True)
            if len(approx) == 3:
                triangles += [approx]

    return img_grey, triangles