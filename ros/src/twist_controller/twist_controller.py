from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy
#import time

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        
        self.yaw_control = YawController(kwargs['wheel_base'],
                                         kwargs['steer_ratio'],
                                         kwargs['min_speed'],
                                         kwargs['max_lat_accel'],
                                         kwargs['max_steer_angle'])
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.wheel_radius = kwargs['wheel_radius']
        self.throttle_pid_init = kwargs['throttle_pid_init']
        self.vel_lp_coeff = kwargs['vel_lp_coeff']
        self.steer_lp_coeff = kwargs['steer_lp_coeff']
        self.throttle_pid = PID(*self.throttle_pid_init)
        self.vel_filter = LowPassFilter(*self.vel_lp_coeff)
        self.steer_filter = LowPassFilter(*self.steer_lp_coeff)
        
        self.time = rospy.get_time()

    def control(self, linear_exp,angular_exp,linear_curr,dbwenb):#**kwargs):
        # TODO: Change the arg, kwarg list to suit your needsf
        
        if dbwenb == False:
             self.throttle_pid.reset()
             #self.cte_controller.reset()
             return 0.,0.,0.
             
        if self.time == 0.0:
             self.time = rospy.get_time()
             return 0, 0, 0
         
        dt = rospy.rostime.get_time() - self.time;
        self.time = rospy.rostime.get_time()
         
 
        #smooth the noisy currrent velocity values
        linear_curr = self.vel_filter.filt(linear_curr) 
 
        #get appropriate vel. diff we need to achieve
        vel_diff = linear_exp - linear_curr
         
        # get steer using yaw controller
        #print(linear_exp, linear_curr, angular_exp)
        steer = self.yaw_control.get_steering(linear_exp, angular_exp, linear_curr) #+ self.cte_controller.step(vel_diff, dt)
        steer = self.steer_filter.filt(steer)
        
        #get throttle value using throttle pid controller: 0<val<1
        throttle = self.throttle_pid.step(vel_diff, dt)
        throttle = max(0.0, min(throttle, 1.0))
 
        #brake
        brake = 0.0
         
        #handle throttle and brake
        if linear_exp == 0.0 and linear_curr < 0.1: #we need to do a complete stop
             brake = 700; 
             throttle = 0.0
        elif throttle < 0.1 and vel_diff < 0.0: # we need to start braking slowly
             decel = max(self.decel_limit, vel_diff)
             brake = min(700, (self.vehicle_mass+(self.fuel_capacity*GAS_DENSITY))*abs(decel)*self.wheel_radius) #torque in Nm
             throttle = 0.0
        
        return throttle, brake, steer
