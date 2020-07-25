#!/usr/bin/env python

# Columbia Engineering
# MECS 4603 - Fall 2017

import math
import numpy
import time
import rospy
import random

from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D
from state_estimator.msg import RobotPose
from state_estimator.msg import SensorData
from state_estimator.msg import Landmark
from state_estimator.msg import LandmarkReading
from state_estimator.msg import LandmarkSet

def create_landmark(x, y):
   l = Landmark()
   l.x = x
   l.y = y
   return l

class Robot(object):

    def __init__(self):

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.vt = 0.0
        self.vrot = 0.0

        self.step_size = 0.01

        self.model_noise_trans = 0.0025
        self.model_noise_rot = 0.005

        self.sensor_noise_range = 0.1
        self.sensor_noise_bearing = 0.05

        self.start_flag = False

        self.landmarks = []
        self.landmarks.append(create_landmark(5,5))
        self.landmarks.append(create_landmark(5,6))
        self.landmarks.append(create_landmark(6,5))        
        self.landmarks.append(create_landmark(-5,5))
        self.landmarks.append(create_landmark(-5,6))
        self.landmarks.append(create_landmark(-6,5))        
        self.landmarks.append(create_landmark(5,-5))
        self.landmarks.append(create_landmark(5,-6))
        self.landmarks.append(create_landmark(6,-5))        
        self.landmarks.append(create_landmark(-5,-5))
        self.landmarks.append(create_landmark(-5,-6))
        self.landmarks.append(create_landmark(-6,-5))        
        self.landmarks.append(create_landmark(5,0))
        self.landmarks.append(create_landmark(-5,0))
        self.landmarks.append(create_landmark(0,5))
        self.landmarks.append(create_landmark(0,-5))
        self.landmarks.append(create_landmark(1,0))

        self.sensing_range = 2.5

        self.pub_pose = rospy.Publisher("/robot_pose", RobotPose, queue_size=1)
        self.pub_sens = rospy.Publisher("/sensor_data", SensorData, queue_size=1)
        self.pub_landmarks = rospy.Publisher("/landmarks", LandmarkSet, queue_size=1)

        rospy.Timer(rospy.Duration(1.0), self.publish_landmarks)
        rospy.Timer(rospy.Duration(self.step_size), self.step)

        rospy.sleep(5)
        rospy.Timer(rospy.Duration(3.0), self.rand_vel)
        self.start_flag = True

    def get_sensor_data(self):
        sens = SensorData()
        sens.vel_trans = self.vt
        sens.vel_ang = self.vrot
        
        for i in range(0,len(self.landmarks)):
            r = math.sqrt( (self.landmarks[i].x-self.x)*(self.landmarks[i].x-self.x) + 
                           (self.landmarks[i].y-self.y)*(self.landmarks[i].y-self.y) )
            if r < self.sensing_range:
                reading = LandmarkReading()
                reading.landmark = self.landmarks[i]
                reading.range = r
                reading.bearing = math.atan2( (self.landmarks[i].y - self.y),
                                              (self.landmarks[i].x - self.x)) - self.theta

                if self.start_flag:
                    reading.range += numpy.random.normal(0.0, self.sensor_noise_range)
                    reading.bearing += numpy.random.normal(0.0, self.sensor_noise_bearing)

                sens.readings.append(reading)

        return sens

    def step(self, event):
        self.x = self.x + self.step_size * self.vt * math.cos(self.theta)
        self.y = self.y + self.step_size * self.vt * math.sin(self.theta)
        self.theta = self.theta + self.step_size * self.vrot

        if self.start_flag:
            self.x += numpy.random.normal(0.0, self.model_noise_trans)
            self.y += numpy.random.normal(0.0, self.model_noise_trans)
            self.theta += numpy.random.normal(0.0, self.model_noise_rot)

        time = rospy.Time.now()
        pose_msg = RobotPose()
        pose_msg.header.stamp = time
        pose_msg.pose.x = self.x
        pose_msg.pose.y = self.y
        pose_msg.pose.theta = self.theta
        self.pub_pose.publish(pose_msg)

        sensor_msg = self.get_sensor_data()
        sensor_msg.header.stamp = time
        self.pub_sens.publish(sensor_msg)

    def publish_landmarks(self,event=None):
        msg = LandmarkSet()
        msg.landmarks = self.landmarks
        self.pub_landmarks.publish(msg)

    def rand_vel(self, event):
        r = math.sqrt(self.x*self.x + self.y*self.y)
        if math.fabs(self.x) < 6 and math.fabs(self.y) < 6:
            self.vt = 0.5 + random.random() * 1.0
            self.vrot = (-math.pi + random.random() * 2 * math.pi) / 5
        else:
            if ((self.x * math.cos(self.theta) + self.y * math.sin(self.theta)) / r < -0.2):
                self.vt = 0.5 + random.random() * 1.0
                self.vrot = 0.0
            else:
                self.vt = 0.0
                self.vrot = (-math.pi + random.random() * 2 * math.pi) / 2
        
if __name__ == '__main__':
    rospy.init_node('mobile_robot_sim', anonymous=True)
    robot = Robot()
    rospy.spin()

