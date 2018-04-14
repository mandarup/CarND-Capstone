#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from light_classification.tl_extractor import TLExtractor
from light_classification.tl_helpers import increase_contrast

import cv2

from scipy.spatial import KDTree
import tf
import time
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.extract_tl = rospy.get_param('~extract_tl')

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.listener = tf.TransformListener()

        self.pose = None
        self.waypoints = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.has_image = None

        self.camera_image = None
        self.lights = []

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        if self.extract_tl:
            rospy.loginfo('Loading Carla CV traffic light components')

            self.bridge = CvBridge()
            self.tl_classifier = TLClassifier()
            self.tl_extractor = TLExtractor()

            sub6 = rospy.Subscriber('/image_color', Image, self.image_cb_tl)
        else:
            sub6 = rospy.Subscriber('/image_color', Image, self.image_cb_notl)

        # Provides the Vehicles Current Position
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        # Provides a complete list of waypoints the car will be following
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        # TODO: verify whether spin function can get a frequency so that we don't run this too often.
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        self.base_waypoints = msg
        if self.waypoints_2d is None:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                        for waypoint in self.base_waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb_notl(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def image_cb_tl(self, msg):
        """Identified in msg whether a traffic light can be founad and what color it has
        """
        rospy.loginfo('tl_detector.image_cb_tl')

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        start = time.time()
        cv_image = increase_contrast(cv_image)
        self.state_count += 1
        lights = self.tl_extractor.extract_traffic_light(cv_image)
        if len(lights) > 0:
            lights_col = [self.tl_classifier.get_classification(img) for img in lights]
            rospy.loginfo('found lights: {} in {}'.format(lights_col, time.time() - start))
            file_name = '/capstone/imgs/camera/test_{}_{}.jpg'.format(self.state_count, lights_col[0])
            cv2.imwrite(file_name, lights[0])
        else:
            file_name = '/capstone/imgs/camera/test_{}_NOLIGHT.jpg'.format(self.state_count)
            cv2.imwrite(file_name, cv_image)

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        return light.state 
        # commented out to skip TL classifier. this gets traffic light state from simulator

        """if(not self.has_image):
            self.prev_light_loc = None
            return False


        #Get classification
        return self.light_classifier.get_classification(cv_image)
        """

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_index = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        
        car_wp_index = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

        #TODO find the closest visible traffic light (if one exists)
        diff = len(self.base_waypoints.waypoints)
        for i, light in enumerate(self.lights):
            line = stop_line_positions[i]
            
            # reuse KDTtree search from waypoint update
            temp_wp_index = self.get_closest_waypoint(line[0], line[1])
            d = temp_wp_index - car_wp_index
            if d >= 0 and d < diff:
                diff = d
                closest_light = light
                line_wp_index = temp_wp_index

        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_index, state

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
