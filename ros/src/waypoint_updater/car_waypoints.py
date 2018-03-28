from collections import namedtuple
import math

CarCoordWayPoint = namedtuple('CarCoordWayPoint', ['x', 'y', 'seq'])
CarPosition = namedtuple('CarPosition', ['x', 'y', 'theta'])


def car_coord_waypoints(position, waypoints):
    car_coord_wp = []
    if (position.x is not None
            and position.y is  not None
            and position.theta is  not None
            and len(waypoints) > 0):

        cos_theta = math.cos(position.theta)
        sin_theta = math.sin(position.theta)

        for wp in waypoints:
            dx = wp.pose.pose.position.x - position.x
            dy = wp.pose.pose.position.y - position.y
            cc_wp_x = dx * cos_theta + dy * sin_theta
            cc_wp_y = -dx * sin_theta + dy * cos_theta
            car_coord_wp.append(CarCoordWayPoint(x=cc_wp_x, y=cc_wp_y,
                                                seq=wp.pose.header.seq))

    return car_coord_wp
