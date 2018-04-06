from styx_msgs.msg import TrafficLight

from .tl_detector import TLDetector

class TLClassifier(object):
    def __init__(self):
        self.tl_detector = TLDetector()
        #TODO load classifier
        pass

    def get_classification(self, image):
        tl_lights = self.tl_detector.extract_traffic_light(image)

        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
