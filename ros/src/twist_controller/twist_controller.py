from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        ## Initialize Throttle Controller as PID:
        kp=0.123757 ## used values from PID project to begin with
        ki=0.0
        kd=0.770457
        mn=0
        mx=0.2
        self.throttle_controller= PID(kp, ki, kd, mn, mx)
        
        ## Initialize yaw controller::
        self.yaw_controller=YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        ## Initialize low pass filter::
        tau=0.5 
        ts=0.2
        self.lpf=LowPassFilter(tau, ts)
        
        ## Initialize parameters::
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.last_time=rospy.get_time()

    def control(self, curr_vel, linear_vel, angular_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        # Reset to stop if dbw is not enabled:
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.,0.,0.
        
        # Filter out the curr_vel to remove high frequency noise:
        curr_vel=self.lpf.filt(curr_vel)
        self.last_vel=curr_vel
        
        # Compute Steering using Yaw Controller :
        steering =self.yaw_controller.get_steering(linear_vel, angular_vel, curr_vel)
        #rospy.logwarn('linera_Vel $s ,current vel %s, angular_vel: %s', 
               #linear_vel, curr_vel, angular_vel)
        # Compute inputs for the Throttle controller step function (error, sample_time):
        vel_err=linear_vel-curr_vel
        sample_time=rospy.get_time()- self.last_time
        
        throttle=self.throttle_controller.step(vel_err, sample_time)
        #Since Throttle is set now, break should be set to zero
        brake=0
        
        # Store curr values of time and velocity into variable
        self.last_time=rospy.get_time()
        self.last_vel=curr_vel
        
        # Check on braking conditions:
        # Assumption here is if, linear_vel is zero and curr velocity is <0.1, we are trying to stop:
        if linear_vel==0 and curr_vel<0.1:
            throttle=0
            brake = 400 # set to max torque to stay at stop
            
        # Assumption -> if vel_error is negative, vehicle is moving faster than expected, we need to decelerate by applying brake
        if throttle<0.1 and vel_err<0:
            throttle=0
            decel=max(vel_err, self.decel_limit)
            brake= abs(decel)*self.vehicle_mass*self.wheel_radius
        
        return throttle, brake, steering