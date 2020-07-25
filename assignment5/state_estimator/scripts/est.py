#!/usr/bin/env python

# Columbia Engineering
# MECS 4602 - Fall 2018

import math
import numpy
import time

import rospy

from state_estimator.msg import RobotPose
from state_estimator.msg import SensorData

class Estimator(object):
    def __init__(self):

        # Publisher to publish state estimate
        self.pub_est = rospy.Publisher("/robot_pose_estimate", RobotPose, queue_size=1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3,1))
        self.P = numpy.zeros((3,3))

        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005

        self.step_size = 0.01

        # Subscribe to command input and sensory output of robot
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)
        
    # This function gets called every time the robot publishes its control 
    # input and sensory output. You must make use of what you know about 
    # extended Kalman filters to come up with an estimate of the current
    # state of the robot and covariance matrix.
    # The SensorData message contains fields 'vel_trans' and 'vel_ang' for
    # the commanded translational and rotational velocity respectively. 
    # Furthermore, it contains a list 'readings' of the landmarks the
    # robot can currently observe
    def estimate(self, sens):
        #### ----- YOUR CODE GOES HERE ----- ####
        u=[sens.vel_trans,sens.vel_ang]
        F=[[1,0,-self.step_size*u[0]*math.sin(self.x[2][0])],
        [0,1,self.step_size*u[0]*math.cos(self.x[2][0])],
        [0,0,1]]
        x_hat = numpy.zeros((3,1))
        x_hat[0][0] = self.x[0][0] + self.step_size*sens.vel_trans*math.cos(self.x[2][0])
        x_hat[1][0] = self.x[1][0] + self.step_size*sens.vel_trans*math.sin(self.x[2][0])
        x_hat[2][0] = self.x[2][0] + self.step_size*sens.vel_ang
        p_hat=numpy.dot(numpy.dot(F,self.P),numpy.transpose(F))+self.V
        if len(sens.readings) > 0:
            H_k = numpy.zeros((2*len(sens.readings),3))
            for i in range(len(sens.readings)):
                range_hat = math.sqrt((x_hat[0] - sens.readings[i].landmark.x)**2 + (x_hat[1] - sens.readings[i].landmark.y)**2)        
                if range_hat>0.1:
                    H_k[2*i][0] = (x_hat[0][0]-sens.readings[i].landmark.x)/(math.sqrt((x_hat[0][0]-sens.readings[i].landmark.x)**2+(x_hat[1][0]-sens.readings[i].landmark.y)**2))
                    H_k[2*i+1][0] = -(x_hat[1][0]-sens.readings[i].landmark.y)/((x_hat[0][0]-sens.readings[i].landmark.x)**2+(x_hat[1][0]-sens.readings[i].landmark.y)**2)
                    H_k[2*i][1] = (x_hat[1][0]-sens.readings[i].landmark.y)/(math.sqrt((x_hat[0][0]-sens.readings[i].landmark.x)**2+(x_hat[1][0]-sens.readings[i].landmark.y)**2))
                    H_k[2*i+1][1] = (x_hat[0][0]-sens.readings[i].landmark.x)/((x_hat[0][0]-sens.readings[i].landmark.x)**2+(x_hat[1][0]-sens.readings[i].landmark.y)**2)
                    H_k[2*i][2] = 0
                    H_k[2*i+1][2] = -1
            
            W = numpy.zeros((2*len(sens.readings),2*len(sens.readings)))
            for i in range(len(sens.readings)):
                W[2*i][2*i] = 0.1
                W[2*i+1][2*i+1] = 0.05

            y_k = numpy.zeros((2*len(sens.readings),1))
            for i in range(len(sens.readings)):
                y_k[2*i][0] = sens.readings[i].range
                y_k[2*i+1][0] = sens.readings[i].bearing

            h = numpy.zeros((2*len(sens.readings),1))
            for i in range(len(sens.readings)):
                h[2*i][0] = math.sqrt((x_hat[0][0]-sens.readings[i].landmark.x)**2+(x_hat[1][0]-sens.readings[i].landmark.y)**2)
                h[2*i+1][0] = math.atan2(sens.readings[i].landmark.y-x_hat[1][0],sens.readings[i].landmark.x-x_hat[0][0])-x_hat[2][0]
            v=y_k-h
            for b in range(len(sens.readings)):
                if v[2*b+1] < -numpy.pi:
                    v[2*b+1][0] += 2*numpy.pi 
                if v[2*b+1] > numpy.pi:
                    v[2*b+1][0] -= 2*numpy.pi
            S = numpy.dot(numpy.dot(H_k,p_hat),numpy.transpose(H_k)) + W
            R = numpy.dot(numpy.dot(p_hat,numpy.transpose(H_k)),numpy.linalg.inv(S))
            x_k_new=x_hat+numpy.dot(R,v)
            P_k_new=p_hat-numpy.dot(numpy.dot(R,H_k),p_hat)
        else:
            x_k_new=x_hat
            P_k_new=p_hat
        self.x=x_k_new
        self.P=P_k_new
    
    def sensor_callback(self,sens):

        # Publish state estimate 
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = self.x[0]
        est_msg.pose.y = self.x[1]
        est_msg.pose.theta = self.x[2]
        self.pub_est.publish(est_msg)

if __name__ == '__main__':
    rospy.init_node('state_estimator', anonymous=True)
    est = Estimator()
    rospy.spin()