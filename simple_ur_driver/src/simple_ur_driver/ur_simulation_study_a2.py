#!/usr/bin/env python
# ROS IMPORTS
import roslib; roslib.load_manifest('simple_ur_driver')
import rospy
import tf; import tf_conversions as tf_c
import PyKDL
# URDF
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
# MSGS and SERVICES
from simple_ur_msgs.srv import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from predicator_msgs.msg import *
from std_msgs.msg import *
import time
import threading
import socket
# URX Universal Robot Driver
import urx
# OTHER
import logging
import numpy as np
from pid import PID

def tr(deg):
    return deg*3.314159265/180.0

class URDriver():
    MAX_ACC = .5
    MAX_VEL = 1.8
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    def __init__(self):
        rospy.init_node('ur_simulation',anonymous=True)
        rospy.logwarn('SIMPLE_UR SIMULATION DRIVER LOADING')
        # TF
        self.broadcaster_ = tf.TransformBroadcaster()
        self.listener_ = tf.TransformListener()
        # SERVICES
        self.servo_to_pose_service = rospy.Service('simple_ur_msgs/ServoToPose', ServoToPose, self.servo_to_pose_call)
        self.set_stop_service = rospy.Service('simple_ur_msgs/SetStop', SetStop, self.set_stop_call)
        self.set_teach_mode_service = rospy.Service('simple_ur_msgs/SetTeachMode', SetTeachMode, self.set_teach_mode_call)
        self.set_servo_mode_service = rospy.Service('simple_ur_msgs/SetServoMode', SetServoMode, self.set_servo_mode_call)
        # PUBLISHERS AND SUBSCRIBERS
        self.driver_status_publisher = rospy.Publisher('/ur_robot/driver_status',String)
        self.robot_state_publisher = rospy.Publisher('/ur_robot/robot_state',String)
        self.joint_state_publisher = rospy.Publisher('joint_states',JointState)
        # self.follow_pose_subscriber = rospy.Subscriber('/ur_robot/follow_goal',PoseStamped,self.follow_goal_cb)
        # Predicator
        self.pub_list = rospy.Publisher('/predicator/input', PredicateList)
        self.pub_valid = rospy.Publisher('/predicator/valid_input', ValidPredicates)
        rospy.sleep(1)
        pval = ValidPredicates()
        pval.pheader.source = rospy.get_name()
        pval.predicates = ['moving', 'stopped', 'running']
        pval.assignments = ['robot']
        self.pub_valid.publish(pval)
        # Rate
        self.run_rate = rospy.Rate(100)
        self.run_rate.sleep()
        ### Set Up Simulated Robot ###
        self.driver_status = 'TEACH'
        self.robot_state = 'POWER OFF'
        robot = URDF.from_parameter_server()
        self.kdl_kin = KDLKinematics(robot, 'base_link', 'ee_link')
        # self.q = self.kdl_kin.random_joint_angles()
        # self.q = [-1.5707,-1.396,-2.356,-2.356,-1.5707,0] # Start Pose?
        self.q = [-2.23101701, -2.25110881, -0.84351126, -3.15477596, -2.25848383, -0.35775537] # Start Pose?
        self.start_pose = self.kdl_kin.forward(self.q)
        self.F_start = tf_c.fromMatrix(self.start_pose)
        # rospy.logwarn(self.start_pose)
        # rospy.logwarn(type(self.start_pose))
        # pose = self.kdl_kin.forward(q)
        # joint_positions = self.kdl_kin.inverse(pose, q+0.3) # inverse kinematics
        # if joint_positions is not None:
        #     pose_sol = self.kdl_kin.forward(joint_positions) # should equal pose
        # J = self.kdl_kin.jacobian(q)
        # rospy.logwarn('q:'+str(q))
        # rospy.logwarn('joint_positions:'+str(joint_positions))
        # rospy.logwarn('pose:'+str(pose))
        # if joint_positions is not None:
        #     rospy.logwarn('pose_sol:'+str(pose_sol))
        # rospy.logwarn('J:'+str(J))

        ### START LOOP ###
        while not rospy.is_shutdown():
            if self.driver_status == 'TEACH':
                self.update_from_marker()
            
            # if self.driver_status == 'SERVO':
            #     self.update_follow()

            # Publish and Sleep
            self.publish_status()
            self.send_command()
            self.run_rate.sleep()

        # Finish
        rospy.logwarn('SIMPLE UR - Simulation Finished')

    def update_from_marker(self):
        try:
            F_target_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/endpoint_interact',rospy.Time(0)))
            F_target_base = tf_c.fromTf(self.listener_.lookupTransform('/base_link','/endpoint_interact',rospy.Time(0)))
            F_base_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/base_link',rospy.Time(0)))
            self.F_command = F_base_world.Inverse()*F_target_world
            M_command = tf_c.toMatrix(self.F_command)

            joint_positions = self.kdl_kin.inverse(M_command, self.q) # inverse kinematics
            if joint_positions is not None:
                pose_sol = self.kdl_kin.forward(joint_positions) # should equal pose
                self.q = joint_positions
            else:
                rospy.logwarn('no solution found')

            # self.send_command()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(str(e))

    def send_command(self):
        # rospy.logwarn(self.q)
        self.current_joint_positions = self.q
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "simuated_data"
        msg.name = self.JOINT_NAMES
        msg.position = self.current_joint_positions
        msg.velocity = [0]*6
        msg.effort = [0]*6
        self.joint_state_publisher.publish(msg)

        pose = self.kdl_kin.forward(self.q)
        F = tf_c.fromMatrix(pose)
        # F = self.F_command
        self.current_tcp_pose = tf_c.toMsg(F)
        self.current_tcp_frame = F
        self.broadcaster_.sendTransform(tuple(F.p),tuple(F.M.GetQuaternion()),rospy.Time.now(), '/endpoint','/base_link')

    def set_teach_mode_call(self,req):
        if req.enable == True:
            # self.rob.set_freedrive(True)
            self.driver_status = 'TEACH'
            return 'SUCCESS - teach mode enabled'
        else:
            # self.rob.set_freedrive(False)
            self.driver_status = 'IDLE'
            return 'SUCCESS - teach mode disabled'

    def set_servo_mode_call(self,req):
        if req.mode == 'SERVO':
            self.driver_status = 'SERVO'
            return 'SUCCESS - servo mode enabled'
        elif req.mode == 'DISABLE':
            self.driver_status = 'IDLE'
            return 'SUCCESS - servo mode disabled'

    def set_stop_call(self,req):
        rospy.logwarn('SIMPLE UR - STOPPING ROBOT')
        self.rob.stop()
        return 'SUCCESS - stopped robot'

    def servo_to_pose_call(self,req): 
        if self.driver_status == 'SERVO':
            rospy.logwarn(req)
            self.F_command = tf_c.fromMsg(req.target)
            M_command = tf_c.toMatrix(self.F_command)
            # Calculate IK
            joint_positions = self.kdl_kin.inverse(M_command, self.q) # inverse kinematics
            if joint_positions is not None:
                pose_sol = self.kdl_kin.forward(joint_positions) # should equal pose
                self.q = joint_positions
            else:
                rospy.logwarn('no solution found')
            # self.send_command(F_command)
            return 'SUCCESS - moved to pose'
        else:
            rospy.logerr('SIMPLE UR -- Not in servo mode')
            return 'FAILED - not in servo mode'

    def publish_status(self):
        self.driver_status_publisher.publish(String(self.driver_status))
        self.robot_state_publisher.publish(String(self.robot_state))

        ps = PredicateList()
        ps.pheader.source = rospy.get_name()
        ps.statements = []

        statement = PredicateStatement( predicate='moving',
                                        confidence=1,
                                        value=PredicateStatement.FALSE,
                                        num_params=1,
                                        params=['robot', '', ''])
        ps.statements += [statement]
        statement = PredicateStatement( predicate='stopped',
                                        confidence=1,
                                        value=PredicateStatement.TRUE,
                                        num_params=1,
                                        params=['robot', '', ''])
        ps.statements += [statement]
        statement = PredicateStatement( predicate='running',
                                        confidence=1,
                                        value=PredicateStatement.TRUE,
                                        num_params=1,
                                        params=['robot', '', ''])
        ps.statements += [statement]
        self.pub_list.publish(ps)

    def check_robot_state(self):
        self.robot_state == 'RUNNING SIMULATION'
        self.driver_status = 'IDLE - SIMULATION'

if __name__ == "__main__":
    robot_driver = URDriver()