import numpy as np
import cv2 as cv

def calibrate_camera(frame):
    """
    Performs image analysis on input image, returns center positions of red objects

    Parameters:
    - frame (numpy.ndarray): array containing BGR values

    Returns:
    - pixel_to_spatial_ratio: pixel to spatial distance ratio calibrated from set reference markers
    """

    gaussian_kernel = np.ones((5,5),np.float32)/25
    gaussian_filter = cv.filter2D(frame,-1,gaussian_kernel)

    x = []
    y = []

    hsv = cv.cvtColor(gaussian_filter, cv.COLOR_BGR2HSV)

    yellow_lower = np.array([25,0,200])
    yellow_upper = np.array([40,255,255])

    yellow_mask = cv.inRange(hsv, yellow_lower, yellow_upper)

    output = cv.connectedComponentsWithStats(yellow_mask, 8, cv.CV_32S)
    (numLabels, labels, stats, centroids) = output

    for id in range(numLabels):
        if stats[id, cv.CC_STAT_WIDTH] < 96/2 and stats[id, cv.CC_STAT_WIDTH] > 76/2:
            if stats[id, cv.CC_STAT_HEIGHT] < 96/2 and stats[id, cv.CC_STAT_HEIGHT] > 76/2:
                x.append(centroids[id][0])
                y.append(centroids[id][1])

    assert len(x) == 2, "Found more/fewer calibration points"

    pixel_to_meter_ratio = 0.335 / abs(x[0]-x[1])

    camera_offset = 0.5 - (720 - y[0]) * pixel_to_meter_ratio 

    return pixel_to_meter_ratio, camera_offset
