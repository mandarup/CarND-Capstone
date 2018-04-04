#!/usr/bin/env python
"""Waypoint Updater.
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO:
    - Stopline location for each traffic light.
"""

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf
from scipy.spatial import KDTree
import numpy as np
import math



LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

MAX_DECEL = 1. # max. allowed deceleration

### Deceleration profile functions:

# Proposal by Udacity-walkthrough for deceleration profile:
def deceleration_sqrt(dist):
    x = 2 * MAX_DECEL * dist
    vel = math.sqrt(x)
    return vel

# Further scaling of profile will be necessary later (still untested, 30/03/2018)
def deceleration_sigmoid(dist):
    x = dist
    vel = 1/(1+math.exp(-x))
    return vel

# Further scaling of profile will be necessary later (still untested 30/03/2018)
def deceleration_atan(dist):
    x = dist - 10.
    vel = math.atan(x) + 0.5 * math.pi
    return vel





class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb) # Provides the Vehicles Current Position
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb) # Provides a complete list of waypoints the car will be following
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb) # Get the position of the closest waypoint to a Red Light Stop

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_pose = None
        self.base_waypoints = None

        self.waypoints_2d = None
        self.waypoint_tree = None

        self.stopline_wp_idx = -1

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # rospy.logdebug("current_pose: {}".format(self.current_pose is not None))
            # rospy.logdebug("base_waypoints: {}".format(self.base_waypoints is not None))
            # rospy.logdebug("waypoint_tree: {}".format(self.waypoint_tree is not None))
            if self.current_pose is not None and self.base_waypoints is not None and self.waypoint_tree is not None:
                #get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y],1)[1]

        #check if closes waypoint is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        
        farthest_idx = closest_idx + LOOKAHEAD_WPS

        lane.waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        # Impose deceleration profile onto waypoints if traffic light is detected 
        # and within range (i.e. nearer than farthest_idx)



        if self.stopline_wp_idx != -1 and (self.stopline_wp_idx < farthest_idx):
            lane.waypoints = self.decelerate(lane.waypoints, closest_idx)

        self.final_waypoints_pub.publish(lane)

    # Imposes deceleration profile if traffic light or object is detected on trajectory
    def decelerate(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            # Deceleration profile is a sqrt function
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # Center of the car will be beind the line rather than right on
            dist = self.distance(waypoints, i, stop_idx)
            vel = deceleration_sqrt(dist)
            # Alternative profile functions (try out later):
            # vel = deceleration_sigmoid(dist)
            # vel = deceleration_atan(dist)

            if vel < 1.:
                vel = 0.

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp


    def pose_cb(self, msg):
        self.current_pose = msg

    def waypoints_cb(self, msg):
        self.base_waypoints = msg
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                        for waypoint in self.base_waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data;
        # pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
