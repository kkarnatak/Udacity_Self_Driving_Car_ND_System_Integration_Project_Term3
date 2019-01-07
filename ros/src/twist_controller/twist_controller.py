from pid import PID
from yaw_controller import YawController
import math

###############################################################################

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

###############################################################################

class Controller(object):
    def __init__(self, *args, **kwargs):

	# Get the parameters relaed to the vehicle


        speed_threshold = 0

	self.brake_deadband = kwargs['brake_deadband']
        self.fuel_capacity = kwargs['fuel_capacity']
        
	self.steer_ratio = kwargs['steer_ratio']
	self.vehicle_mass = kwargs['vehicle_mass']

        self.wheel_radius = kwargs['wheel_radius']
        self.wheel_base = kwargs['wheel_base']
        
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']

        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']


        self.long_pid = PID(kp=0.7, ki=0.001, kd=0.08, mn=self.decel_limit, mx=0.5 * self.accel_limit)
	
	# Instantiate the PID controllers
        self.steering_pid = PID(kp=0.15, ki=0.001, kd=0.1, mn=-self.max_steer_angle, mx=self.max_steer_angle)
        #self.long_pid = PID(kp=0.8, ki=0, kd=0.05, mn=self.decel_limit, mx=0.5 * self.accel_limit)

	# Instantiate the Yaw controller
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, speed_threshold, \
		self.max_lat_accel, self.max_steer_angle)


    ##############################################################################################

    def control(self, desired_velocity, desired_angular_velocity, current_velocity, _cte, duration):

        e_velocity = desired_velocity - current_velocity

        delta_velocity = self.long_pid.step(e_velocity, duration)

        brake = 0
	total_mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY

	# The throttle should be set as per the different in the veloctiy computed from the error in the
	# desired and current velocity
        throttle = delta_velocity

        if(throttle < 0):
	    # If throttle negative, set deceleration
            ego_deceleration = abs(throttle)
	    
            if ego_deceleration > self.brake_deadband:
            	brake = total_mass * self.wheel_radius * ego_deceleration
	    else:
		brake = 0.

            throttle = 0

	pid_steering_value = self.steering_pid.step(_cte, duration)
        pred_steering_value = self.yaw_controller.get_steering(desired_velocity, desired_angular_velocity, current_velocity)
        

        steering = pred_steering_value + pid_steering_value

        return throttle, brake, steering

    ##############################################################################################

    def reset(self):
	# Reset the parameters when requested from dbw_node
        self.long_pid.reset()
        self.steering_pid.reset()
    ##############################################################################################
