
import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704

throttle_KP = 1
throttle_KI = 0.1
throttle_KD = 0.0

steering_KP = 1
steering_KI = 0.1
steering_KD = 0.0


class Controller(object):
    def __init__(self, vehicle_mass, wheel_radius, accel_limit, decel_limit,
                 wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement

        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit

        self.yaw_controller = YawController(
            wheel_base,
            steer_ratio,
            min_speed,
            max_lat_accel,
            max_steer_angle)
        self.throttle_pid = PID(
            throttle_KP,
            throttle_KI,
            throttle_KD,
            mn=0.0,
            mx=1.0)
        self.steering_pid = PID(
            steering_KP,
            steering_KI,
            steering_KD,
            mn=-max_steer_angle,
            mx=max_steer_angle)
        self.filter = LowPassFilter(0.2, 0.1)

    def control(self, proposed_linear_velocity,
                proposed_angular_velocity, current_linear_vel, dbw_enabled):

        # USE Yaw Controller to convert (and Low pass filter) to control
        # implement Controller to control velocity
        # Using a PID controller for high frequency signals (like noise) is
        # disastrous due to Differential sections amplitude being tied directly
        # to the frequency of the signal

        # proposed_linear_velocity = self.filter.filt(proposed_linear_velocity)
        # proposed_angular_velocity = self.filter.filt(proposed_angular_velocity)
        # current_linear_vel = self.filter.filt(current_linear_vel)

        throttle = brake = steering = 0.

        # Brake is in torgue (N*m) (computed using desired acceleration, weight of the vehicle, and wheel radius)
        # You can calculate the net force acting upon the car
        # Force Forward - vehicle_mass*acceleration (make sure it is
        # decellerating)

        vel_diff = proposed_linear_velocity - current_linear_vel  # also the error

        time_diff = 1. / 50.  # 50Hz total

        accel = vel_diff / time_diff

        if accel < 0.:
            # we are decelerating
            if accel < self.decel_limit:
                accel = self.decel_limit

            # Force required for deceleration
            force_req = abs(accel) * self.vehicle_mass
            # Torque is force * distance from point of rotation
            brake = force_req * self.wheel_radius
        elif accel > 0.:

            # Notes - Throttle is in the Range of 0 to 1 (1 means fully engaged)
            # should do ( (Max_Vel - curr_vel)/Max_vel)
            # max_vel = current_linear_vel + max accel* time_diff

            # vel_diff = min (vel_diff, self.accel_limit*time_diff)

            # vel_diff = self.filter.filt(vel_diff)
            accel = vel_diff / time_diff

            accel = self.filter.filt(accel)

            throttle = self.throttle_pid.step(accel, time_diff)

            if not dbw_enabled:
                self.throttle_pid.reset()

        steering = self.yaw_controller.get_steering(
            proposed_linear_velocity,
            proposed_angular_velocity,
            current_linear_vel)

        # steering = self.filter.filt(steering)

        steering = self.steering_pid.step(steering, time_diff)

        if not dbw_enabled:
            self.steering_pid.reset()

        # Return throttle, brake, steer
        return throttle, brake, steering
