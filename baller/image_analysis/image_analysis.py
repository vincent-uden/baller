import numpy as np
import cv2 as cv

def get_target_position(frame):
    """
    Performs image analysis on input image, returns center positions of red objects

    Parameters:
    - frame (numpy.ndarray): array containing BGR values

    Returns:
    - x_pos: array containing pixel x-coordinates of red objects
    - y_pos: array containing pixel y-coordinates of red objects
    """

    x_pos = []
    y_pos = []

    gaussian_kernel = np.ones((5,5),np.float32)/25
    gaussian_filter = cv.filter2D(frame,-1,gaussian_kernel)

    hsv = cv.cvtColor(gaussian_filter, cv.COLOR_BGR2HSV)

    red_lower1 = np.array([0,170,150])
    red_upper1 = np.array([5,255,255])

    red_lower2 = np.array([170,170,150])
    red_upper2 = np.array([179,255,255])

    red_mask1 = cv.inRange(hsv, red_lower1, red_upper1)
    red_mask2 = cv.inRange(hsv, red_lower2, red_upper2)
    red_mask = red_mask1 | red_mask2

    # cv.imshow("red mask", red_mask)
    # cv.waitKey(100)

    output = cv.connectedComponentsWithStats(red_mask, 8, cv.CV_32S)
    (numLabels, labels, stats, centroids) = output
    
    for id in range(numLabels):
        if stats[id, cv.CC_STAT_WIDTH] < 300 and stats[id, cv.CC_STAT_WIDTH] > 100:
            if stats[id, cv.CC_STAT_HEIGHT] < 300 and stats[id, cv.CC_STAT_HEIGHT] > 100:
                x_pos.append(centroids[id][0])
                y_pos.append(centroids[id][1])


    return x_pos, y_pos


def get_magazine_count(frame):
    """
    Performs image analysis on input image, returns the number of green objects

    Parameters:
    - frame (numpy.ndarray): array containing BGR values

    Returns:
    - magazine_count: integer equal to number of green obects found
    """

    magazine_count = 0

    gaussian_kernel = np.ones((5,5),np.float32)/25
    gaussian_filter = cv.filter2D(frame,-1,gaussian_kernel)

    hsv = cv.cvtColor(gaussian_filter, cv.COLOR_BGR2HSV)

    green_lower = np.array([70, 40,20])
    green_upper = np.array([90, 255,255])

    green_mask = cv.inRange(hsv, green_lower, green_upper)


    output = cv.connectedComponentsWithStats(green_mask, 8, cv.CV_32S)
    (numLabels, labels, stats, centroids) = output

    # cv.rectangle(green_mask, (0,0), (50, 30), (255,))
    # cv.imshow("magazine mask", green_mask)
    # cv.waitKey(100)
    
    for id in range(numLabels):
        if stats[id, cv.CC_STAT_WIDTH] < 200 and stats[id, cv.CC_STAT_WIDTH] > 40:
            if stats[id, cv.CC_STAT_HEIGHT] < 80 and stats[id, cv.CC_STAT_HEIGHT] > 15:
                magazine_count += 1

    if magazine_count > 1:
        magazine_count += 1

    return magazine_count


if __name__ == '__main__':
    camera = cv.VideoCapture(0)
    while True:
        ret, frame = camera.read()
        try:
            get_target_position(frame)
            if cv.waitKey(10) & 0xff == ord('q'):
                break
        except:
            break
