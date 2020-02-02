#!/usr/bin/env python
import roslib; roslib.load_manifest('rover_driver_base')
import rospy
from geometry_msgs.msg import Twist
import numpy
from numpy.linalg import pinv
from math import atan2, hypot, pi, cos, sin

prefix=["FL","FR","CL","CR","RL","RR"]

class RoverMotors:
    def __init__(self):
        self.steering={}
        self.drive={}
        for k in prefix:
            self.steering[k]=0.0
            self.drive[k]=0.0
    def copy(self,value):
        for k in prefix:
            self.steering[k]=value.steering[k]
            self.drive[k]=value.drive[k]

class DriveConfiguration:
    def __init__(self,radius,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius

class RoverKinematics:
    def __init__(self):
        self.X = numpy.asmatrix(numpy.zeros((3,1)))
        self.motor_state = RoverMotors()
        self.first_run = True

    def twist_to_motors(self, twist, drive_cfg, skidsteer=True, drive_state=None):
        motors = RoverMotors()
        if skidsteer:
            for k in drive_cfg.keys():
                # Insert here the steering and velocity of 
                # each wheel in skid-steer mode
                motors.steering[k] = 0
		if k=="FL" or k=="CL" or k=="RL":
               		motors.drive[k] = twist.linear.x/drive_cfg[k].radius-twist.angular.z
		else:
			motors.drive[k] = twist.linear.x/drive_cfg[k].radius+twist.angular.z
        else:
            for k in drive_cfg.keys():
                # print("%s: %f %f"%(k,drive_state.steering[k],drive_state.drive[k]))
                # Insert here the steering and velocity of 
                # each wheel in rolling-without-slipping mode
                #motors.drive[k] = -(twist.linear.x/drive_cfg[k].radius)
		vx =twist.linear.x - twist.angular.z*drive_cfg[k].y
		vy =twist.linear.y + twist.angular.z*drive_cfg[k].x
		if atan2(vy,vx)>=-pi/2 and atan2(vy,vx)<=pi/2:
			motors.steering[k] = atan2(vy,vx)
			motors.drive[k] = hypot(vy,vx)/drive_cfg[k].radius
		else :
			motors.steering[k] = atan2(vy,vx)-pi
			motors.drive[k] = -hypot(vy,vx)/drive_cfg[k].radius
		
		
        return motors

    def integrate_odometry(self, motor_state, drive_cfg):
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            return self.X
        # Insert here your odometry code
        self.X[0,0] += 0.0
        self.X[1,0] += 0.0
        self.X[2,0] += 0.0
        self.motor_state.copy(motor_state)
        return self.X



