#!/usr/bin/env python

import numpy
import random
import sys
import math
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''    
    def motion_planning(self, ee_goal):
#        print "Starting motion planning"
	########INSERT YOUR RRT MOTION PLANNING HERE##########
        xyz = numpy.array([ee_goal.translation.x, ee_goal.translation.y, ee_goal.translation.z])
        quaternion = numpy.array([ee_goal.rotation.x, ee_goal.rotation.y, ee_goal.rotation.z, ee_goal.rotation.w])
        Trans = tf.transformations.translation_matrix(xyz)
        Rot = tf.transformations.quaternion_matrix(quaternion)
        b_T_ee_d = numpy.dot(Trans, Rot)
        q_d = numpy.array(self.IK(b_T_ee_d))
        for i in range(len(q_d)):
            if q_d[i] > math.pi:
                q_d[i] = q_d[i] - 2 * math.pi
            elif q_d[i] < -math.pi:
                q_d[i] = q_d[i] + 2 * math.pi

        q_c = numpy.array(self.q_current)
        q_d_class = RRTBranch(q_d, q_d)
        point_set = []
        point_set.append(q_d_class)
        q_rand = numpy.array([0.0 for i in range(self.num_joints)])
        time = 0
        while True:
            while True:

                q_d_estim = numpy.array([0.0 for i in range(self.num_joints)])
                #produce a random list

                while True:
                    for i in range(self.num_joints):
                        q_rand[i] = random.uniform(- math.pi, math.pi)
                    if self.is_state_valid(q_rand):
                        break

                #the distance between random point and start point
                l = 0.0
                for i in range(self.num_joints):
                    l = l + (point_set[0].q[i] - q_rand[i]) ** 2
                l = math.sqrt(l)

                near_point = point_set[0]
                #find the nearest point
                for point in point_set:
                    d = 0.0
                    for i in range(len(point.q)):
                        d = d + (point.q[i] - q_rand[i]) ** 2
                    d = math.sqrt(d)
                    if d <= l:
                        l = d
                        near_point = point
                dq = q_rand - near_point.q

                q_d_estim = near_point.q + 0.1 / l * dq



                #find new point
                if self.is_state_valid(q_d_estim):
                    break

            
            #put new point in point set
            newpoint = RRTBranch(near_point.q, q_d_estim)
            newpoint.q = numpy.array(newpoint.q)
            point_set.append(newpoint)
            #test if new point is able to get goal
            if self.is_segment_valid(q_c, newpoint.q):
                point_set.append(RRTBranch(newpoint.q, q_c))
                break

        #find valid path
        validpath = []
        validpath.append(point_set[-1])

        j = 0
        i = 0
        while True:
            for i in range(len(point_set)):

                if (point_set[i].q == validpath[j].parent).all():
                    validpath.append(point_set[i])
                    
                    break
            if (validpath[-1].q == q_d).all() and (validpath[0].q == q_c).all():

                break
            j = j + 1 
   
        #simplify valid path
        simppath = []
        simppath.append(validpath[0].q)
        j = 0
        i = 1
        short = False
        while True:
            while True:

                if self.is_segment_valid(validpath[j].q,validpath[i].q):
                    i = i + 1
                else:
                    simppath.append(validpath[i - 1].q)
                    j = i - 1
                    break
                if i == len(validpath):
                    simppath.append(validpath[i - 1].q)
                    short = True
                    break
            if short:
                break

        #sample
        path = []
        path.append(simppath[0])
        for i in range(len(simppath) - 1):
            point_dis = 0.0
            for j in range(self.num_joints):
                point_dis = point_dis + (simppath[i][j] - simppath[i + 1][j])** 2 
            point_dis = math.sqrt(point_dis)
            if point_dis <= 0.5:
                path.append(simppath[i + 1])
                continue
            else:
                part = int(point_dis / 0.5)
                part = part + 1
                dq_2 = simppath[i + 1] - simppath[i]
                newp = simppath[i]
                for _ in range(part - 1):
                    newp = newp + dq_2 / part
                    path.append(newp)
                path.append(simppath[i + 1])
        
        for i in range(len(path)):
            path[i] = path[i].tolist()
        
        pubjoint = JointTrajectory()

        pubjoint.joint_names = self.joint_names
        for i in range(len(path)):
            
            pubpoint = JointTrajectoryPoint()
            pubpoint.positions = path[i]
            pubjoint.points.append(pubpoint)
            self.pub.publish(pubjoint)
        

        ######################################################
    def is_segment_valid(self,q_parent, q_child):
            dq = q_child - q_parent
            distan_dq = 0
            for i in dq:
                distan_dq = distan_dq + i * i
            distan_dq = math.sqrt(distan_dq)
            part = int(distan_dq / 0.1)
            part = part + 1

            q_test = q_parent
            test = True
            for i in range(part):
                q_test = q_test + dq / part
                if not self.is_state_valid(q_test):
                    test = False
                    break
            return test






    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
	self.parent = parent
	self.q = q


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

