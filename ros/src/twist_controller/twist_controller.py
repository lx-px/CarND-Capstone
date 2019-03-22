from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy
import time

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.throttle_pid = PID(0.5,0.02,0.2)
        self.filter = LowPassFilter(0.2,0.1)
        self.yaw_control = YawController(kwargs["wheel_base"],
                                         kwargs['steer_ratio'],
                                         kwargs['max_lat_accel'],
                                         kwargs['max_steer_angle'],
                                         kwargs['max_steer_angle'])
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.wheel_radius = kwargs['wheel_radius']
        
        self.time = 0.0

    def control(self, linear_exp,angular_exp,linear_curr,dbwenb):#**kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
         print(linear_exp,angular_exp,linear_curr,dbwenb)
         
         if self.time == 0.0:
             self.time = rospy.get_time()
             print(self.time)
             return 0, 0, 0
         #linear_exp = kwargs["linear_exp"]
         #angular_exp = kwargs["angular_exp"]
         #linear_curr = kwargs["linear_curr"]
         #dbwenb = kwargs["dbwenb"]
             
         
         #time diff
         dt = rospy.rostime.get_time() - self.time;
         self.time = rospy.rostime.get_time()
         print(dt, rospy.rostime.get_time(), time.time())
 
         # get steer using yaw controller
         steer = self.yaw_control.get_steering(linear_exp, angular_exp, linear_exp)
         steer = self.filter.filt(steer)
 
         #get appropriate vel. diff we need to achieve
         vel_diff = linear_exp - linear_curr
         print(vel_diff, self.decel_limit*dt, self.accel_limit*dt)
         vel_diff = max(self.decel_limit*dt, min(self.accel_limit*dt, vel_diff)) #max. vel achievable using out accel/decl limits
         print(vel_diff)
 
         #get throttle value using throttle pid controller: 0<val<1
         throttle = self.throttle_pid.step(vel_diff, dt)
         throttle = max(0.0, min(throttle, 1.0))
 
         #brake
         if vel_diff < 0: #we need to brake
             brake = (self.vehicle_mass+(self.fuel_capacity*GAS_DENSITY))*(-vel_diff/dt)*self.wheel_radius #torque in Nm
             throttle = 0.0
         else:
             brake = 0.0

         print(throttle, brake, steer)
         return throttle, brake, steer
