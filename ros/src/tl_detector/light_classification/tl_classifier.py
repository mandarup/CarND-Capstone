import cv2
import numpy as np
from styx_msgs.msg import TrafficLight

UNKNOWN = TrafficLight.UNKNOWN
GREEN = TrafficLight.GREEN
YELLOW = TrafficLight.YELLOW
RED = TrafficLight.RED
IMG_WIDTH, IMG_HEIGHT = 3 * 20, 20

class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        img = cv2.resize(image, (IMG_HEIGHT, IMG_WIDTH), interpolation=cv2.INTER_CUBIC)
        highest_mean = 0
        #TODO use global names instead of local
        tl_col = UNKNOWN
        for i, col in enumerate([RED, YELLOW, GREEN]):
            box_mean = np.mean(img[i * 20: ((i + 1) * 20 - 1), :])
            if box_mean > highest_mean:
                highest_mean = box_mean
                tl_col = col

        return tl_col
