#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb) # Provides the Vehicles Current Position
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb) # Provides a complete list of waypoints the car will be following

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.current_pose = None
        self.all_waypoints = None

        self.loop()


    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            if self.current_pose is not None and self.all_waypoints is not None:
                quat = self.current_pose.pose.orientation
                quat_array = [quat.x, quat.y, quat.z, quat.w]
                theta = tf.transformations.euler_from_quaternion(quat_array)[2]

                # Now that we have an update on position we can determine which waypoints are ahead of us
                cp = CarPosition(self.current_pose.pose.position.x, self.current_pose.pose.position.y, theta)
                # Calculate car relative waypoints
                rel_wp = car_coord_waypoints(cp, self.all_waypoints)
                # Create a list that contains the car relative waypoints and the index of the waypoints
                wps = [[rel, idx] for idx, rel in enumerate(rel_wp)]
                # Remove all waypoints that are not in front of us
                wps = list(filter(lambda wp: wp[0].y > 0, wps))
                # Sort the waypoints on distance to us
                sorted_wps = sorted(wps, key=lambda wp: math.sqrt((wp[0].x - cp.x)**2 + (wp[0].y - cp.y)**2))
                # Backout the original waypoints
                orig_waypoints = [self.all_waypoints[i[1]] for i in sorted_wps]

                final_lane = Lane()
                final_lane.header.frame_id = '/world'
                final_lane.header.stamp = rospy.Time().now()
                final_lane.waypoints = orig_waypoints[:LOOKAHEAD_WPS]

                # TODO: add setting velocity of waypoints

                self.final_waypoints_pub.publish(final_lane)
            rate.sleep()


    def pose_cb(self, msg):
        self.current_pose = msg


    def waypoints_cb(self, lane):
        final_lane = Lane()
        final_lane.header.frame_id = '/final_waypoints'
        final_lane.header.stamp = rospy.Time(0)
        final_lane.waypoints = lane.waypoints[:LOOKAHEAD_WPS]

        self.final_waypoints_pub.publish(final_lane)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
