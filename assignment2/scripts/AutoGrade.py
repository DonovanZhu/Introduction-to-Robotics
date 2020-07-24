#!/usr/bin/env python
import sys
import rospy
import tf
from sys_urdf_setup import *

def talker(msg):
    pub.publish(msg)

def timeout(event):
    global done
    print "Did not receive valid transforms. Grader timed out."
    print "Base Link Transform: ", 0
    print "Link 3 Transform: ", 0
    print "Link 7 Transform: ", 0
    done = True

def print_output(trans, rot, T, link_name, value, grade):
            correct = False
            if (abs(trans - tf.transformations.translation_from_matrix(T)) < 10e-5).all():
                if (abs(rot - tf.transformations.quaternion_from_matrix(T)) < 10e-5).all():
                    correct = True
            if correct:
                print "%s Transform: PASSED"%link_name
                print "%s Transform: "%link_name, value
                grade += value
            else:
                print "%s Transform: FAILED"%link_name
                print "%s Transform: "%link_name, 0

            return grade

if __name__ == '__main__':
    rospy.init_node('project2_grader')
    pub = rospy.Publisher("joint_states", JointState, queue_size=10)
    rospy.sleep(0.5)
    rospy.Timer(rospy.Duration(10), timeout, True)
    all_transforms, transform_offset, robot, done, msg, base_link = l1ll111ll_opy_()
    while not done:
	listener = tf.TransformListener()
        talker(msg)
        rospy.sleep(1)
        try:
            grade = 0
            transform_to_test = 0
            T = all_transforms[transform_to_test]
            (trans, rot) = listener.lookupTransform(base_link, robot.links[transform_to_test+transform_offset].name, rospy.Time(0))
            grade = print_output(trans, rot, T, "Base Link",3, grade)            

            transform_to_test = 3
            T = all_transforms[transform_to_test]
            (trans, rot) = listener.lookupTransform(base_link, robot.links[transform_to_test+transform_offset].name, rospy.Time(0))
            grade = print_output(trans, rot, T, "Link 3",3, grade)            

            transform_to_test = 7
            T = all_transforms[transform_to_test]
            (trans, rot) = listener.lookupTransform(base_link, robot.links[transform_to_test+transform_offset].name, rospy.Time(0))
            grade = print_output(trans, rot, T, "Link 7",4, grade)            

            print "Grade: ", grade
            done = True
        except tf.Exception:
            continue
