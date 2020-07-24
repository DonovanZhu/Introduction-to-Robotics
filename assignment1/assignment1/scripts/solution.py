#!/usr/bin/env python  
import rospy
import math
import numpy
import tf
import tf2_ros
import geometry_msgs.msg
from tf import transformations as t

def publish_transforms():

    #Transform from BASE frame to OBJECT frame
    b2o = geometry_msgs.msg.TransformStamped()
    b2o.header.stamp = rospy.Time.now()
    b2o.header.frame_id = "base_frame"
    b2o.child_frame_id = "object_frame"

    q1 = tf.transformations.quaternion_from_euler(0.64, 0.64, 0.0)
    R_b2o = tf.transformations.quaternion_matrix(q1)
    trans = [1.5,0.8,0]
    T_b2o = t.translation_matrix(trans)
    bTo = numpy.dot(R_b2o,T_b2o)
    trans_new_bTo = t.translation_from_matrix(bTo)

    b2o.transform.rotation.x = q1[0]
    b2o.transform.rotation.y = q1[1]
    b2o.transform.rotation.z = q1[2]
    b2o.transform.rotation.w = q1[3]
    b2o.transform.translation.x = trans_new_bTo[0]
    b2o.transform.translation.y = trans_new_bTo[1]
    b2o.transform.translation.z = trans_new_bTo[2]
    br.sendTransform(b2o)

    #Transform from BASE frame to ROBOT frame
    b2r = geometry_msgs.msg.TransformStamped()
    b2r.header.stamp = rospy.Time.now()
    b2r.header.frame_id = "base_frame"
    b2r.child_frame_id = "robot_frame"

    q2 = tf.transformations.quaternion_about_axis(1.5, (0,1,0))
    R_b2r = tf.transformations.quaternion_matrix(q2)
    trans = [0.0,0.0,-2.0]
    T_b2r = t.translation_matrix(trans)
    bTr = numpy.dot(R_b2r,T_b2r)
    trans_new_bTr = t.translation_from_matrix(bTr)

    b2r.transform.rotation.x = q2[0]
    b2r.transform.rotation.y = q2[1]
    b2r.transform.rotation.z = q2[2]
    b2r.transform.rotation.w = q2[3]
    b2r.transform.translation.x = trans_new_bTr[0]
    b2r.transform.translation.y = trans_new_bTr[1]
    b2r.transform.translation.z = trans_new_bTr[2]
    br.sendTransform(b2r)
  
    '''
    Calculation:
    The origin of object frame in base frame is trans_new_bTo
    Now change this point from base frame to the robot frame which is 
    translated along x,y,z axes (0.3,0.0,0.3), name this frame "c1"
    '''
    trans_rTc1 = [0.3, 0.0, 0.3]
    T_r2c1 = t.translation_matrix(trans_rTc1)
    #Turn the position of object frame into homogeneous coordinates
    homo_pos = numpy.array(bTo[:,3]).T
    T_c12r = numpy.linalg.inv(T_r2c1)
    T_r2b = numpy.linalg.inv(bTr)
    c1Tb = numpy.dot(T_c12r,T_r2b)
    pos_in_c1 = numpy.dot(c1Tb,homo_pos)

    '''
    Now we know a vector(vo) from the origin of c1 frame to the of 
    rigin of object frame. By calculating the angle between x axis
    of c1 frame and this vector, we can rotate c1 frame along a axis 
    which is vertical to vector vo and x axis.
    '''
    
    vx = [1, 0, 0]
    vo = [pos_in_c1[0], pos_in_c1[1], pos_in_c1[2]]
    theta = math.acos((numpy.vdot(vx,vo))/(math.sqrt(vx[0] * vx[0] + vx[1] * vx[1] + vx[2] * vx[2]) * \
    math.sqrt(vo[0] * vo[0] + vo[1] * vo[1] + vo[2] * vo[2])))
    z = math.sqrt(1/(1 + ((vo[2] * vo[2])/(vo[1] * vo[1]))))
    y = -math.sqrt(1 - z * z)
    x = 0
    R_axis = (x,y,z)
    bTc1 = numpy.dot(bTr,T_r2c1)

#Transform from ROBOT frame to CAMERA frame
    r2c = geometry_msgs.msg.TransformStamped()
    r2c.header.stamp = rospy.Time.now()
    r2c.header.frame_id = "robot_frame"
    r2c.child_frame_id = "camera_frame"
    r2c.transform.translation.x = 0.3
    r2c.transform.translation.y = 0.0
    r2c.transform.translation.z = 0.3
    q3 = tf.transformations.quaternion_about_axis(theta, R_axis)
    r2c.transform.rotation.x = q3[0]
    r2c.transform.rotation.y = q3[1]
    r2c.transform.rotation.z = q3[2]
    r2c.transform.rotation.w = q3[3]
    br.sendTransform(r2c)
    
if __name__ == '__main__':
    rospy.init_node('solution')

    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        publish_transforms()
        rate.sleep()
