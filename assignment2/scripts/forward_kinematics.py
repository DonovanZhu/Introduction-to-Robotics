#!/usr/bin/env python

import numpy
import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf.msg
from urdf_parser_py.urdf import URDF
from tf import transformations as tr

def convert_to_message(T, child, parent):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child
    translation = tf.transformations.translation_from_matrix(T)
    rotation = tf.transformations.quaternion_from_matrix(T)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]        
    return t
    
#Our main class for computing Forward Kinematics
class ForwardKinematics(object):

    #Initialization
    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)

    def callback(self, joint_values):

        root = self.robot.get_root()
        link = [0 for _ in range(9)]
        link[0] = root
        joint = [0 for _ in range(8)]

        for i in range(8):
            (joint[i],link[i + 1]) = self.robot.child_map[link[i]][0]

        Tx = [0 for _ in range(8)]
        for j in range(8):
            i = 0
            while True :
                L = self.robot.joints[i]
                if L.name == joint[j]:
                    rpy = L.origin.rpy
                    xyz = L.origin.xyz
                    Tran = tr.translation_matrix(xyz)
                    R = tf.transformations.euler_matrix(rpy[0], rpy[1], rpy[2],'sxyz')
                    t = numpy.dot(Tran,R)
                    for k in range(len(joint_values.position)):
                        if L.name == joint_values.name[k]:
                            axis = numpy.trunc(L.axis)
                            Rot = tf.transformations.rotation_matrix(joint_values.position[k], (axis[0],axis[1],axis[2]))
                            t = numpy.dot(t,Rot)
                    if j != 0:
                        t = numpy.dot(Tx[j - 1],t)
                    Tx[j] = t
                    break
                i += 1
            tx = convert_to_message(t, link[j+1], root)
            self.pub_tf.publish(tf.msg.tfMessage([tx]))
        
if __name__ == '__main__':
    rospy.init_node('fwk')
    fwk = ForwardKinematics()
    rospy.spin()

