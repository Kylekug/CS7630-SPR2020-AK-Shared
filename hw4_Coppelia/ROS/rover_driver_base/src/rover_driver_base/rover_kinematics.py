#!/usr/bin/env python
import roslib; roslib.load_manifest('rover_driver_base')
import rospy
from geometry_msgs.msg import Twist
import numpy
from numpy.linalg import pinv
from math import atan2, hypot, pi, cos, sin
import time

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

    def angleNormalize(self, ang):
        #helper function to normalize angle b/n 0 -> 2pi
        while(ang < -pi):
            ang += 2*pi
        while(ang > pi):
            ang -= 2*pi
        return ang	


    def twist_to_motors(self, twist, drive_cfg, skidsteer=False, drive_state=None):
        motors = RoverMotors()
        if skidsteer:
            for k in drive_cfg.keys():
                # Insert here the steering and velocity of 
                # each wheel in skid-steer mode
                motors.steering[k] = 0
                if k=="FL" or k=="CL" or k=="RL":
                    motors.drive[k] = (twist.linear.x)/drive_cfg[k].radius-twist.angular.z
                else:
                    motors.drive[k] = (twist.linear.x)/drive_cfg[k].radius+twist.angular.z
        else:
            for k in drive_cfg.keys():
                # print("%s: %f %f"%(k,drive_state.steering[k],drive_state.drive[k]))
                # Insert here the steering and velocity of 
                # each wheel in rolling-without-slipping mode
                vx =twist.linear.x - twist.angular.z*drive_cfg[k].y
                vy =twist.linear.y + twist.angular.z*drive_cfg[k].x
                if atan2(vy,vx)>=-pi/2 and atan2(vy,vx)<=pi/2:
                    motors.steering[k] = atan2(vy,vx)
                    motors.drive[k] = hypot(vy,vx)/drive_cfg[k].radius		
                else:
                    motors.steering[k] = atan2(vy,vx)-pi
                    motors.drive[k] = -hypot(vy,vx)/drive_cfg[k].radius
        return motors


    def integrate_odometry(self, motor_state, drive_cfg):
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.t_i = time.time()
            if self.t_i == 0:
               print "Simulated time not initiated; odometry may malfunction!"

            return self.X

        if self.t_i == 0:
           print "Simulated time still not initialized; odometry function BROKEN!"
        # Insert here your odometry code
        t_d = time.time() - self.t_i
        A=numpy.asmatrix(numpy.zeros((12,3)))
        B=numpy.asmatrix(numpy.zeros((12,1)))
        for k in drive_cfg.keys():
            lastAngle = self.angleNormalize(self.motor_state.drive[k])
            curAngle = self.angleNormalize(motor_state.drive[k])
            difAngle = self.angleNormalize(curAngle - lastAngle)
            s = difAngle * drive_cfg[k].radius
            A[2*drive_cfg.keys().index(k),0]=1
            A[2*drive_cfg.keys().index(k),2]=-drive_cfg[k].y
            A[2*drive_cfg.keys().index(k)+1,1]=1
            A[2*drive_cfg.keys().index(k)+1,2]=drive_cfg[k].x
            B[2*drive_cfg.keys().index(k),0]= s*cos(self.motor_state.steering[k])
            B[2*drive_cfg.keys().index(k)+1,0]= s*sin(self.motor_state.steering[k])
            #print("difAngle = ", difAngle, " | lastAngle = ", lastAngle, " | curAngle = ", curAngle)
        C=pinv(A)*B
        self.X[0,0] += C[0,0]*cos(self.X[2,0])-C[1,0]*sin(self.X[2,0])
        self.X[1,0] += C[0,0]*sin(self.X[2,0])+C[1,0]*cos(self.X[2,0])
        self.X[2,0] += C[2,0]
        # self.X[0,0] += 0.0
        # self.X[1,0] += 0.0
        # self.X[2,0] += 0.0
        self.motor_state.copy(motor_state)

        #print "integrate_odometry just ran!"
        #print "X[0]: " + self.X[0,0] + "; X[1]: " + self.X[1,0] + "; X[2]: " + self.X[2,0]
        return self.X 




