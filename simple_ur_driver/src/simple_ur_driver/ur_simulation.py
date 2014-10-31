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
        # Rate
        self.run_rate = rospy.Rate(30)

        ### Set Up Simulated Robot ###
        self.driver_status = 'SIMULATION'
        self.robot_state = 'POWER OFF'
        robot = URDF.from_parameter_server()
        self.kdl_kin = KDLKinematics(robot, 'base_link', 'ee_link')
        # self.q = self.kdl_kin.random_joint_angles()
        self.q = [-1.5707,-.785,-3.1415+.785,-1.5707-.785,-1.5707,-3.1415]
        self.start_pose = self.kdl_kin.forward(self.q)
        self.F_start = tf_c.fromMatrix(self.start_pose)
        # rospy.logwarn(self.start_pose)
        # rospy.logwarn(type(self.start_pose))
        # pose = self.kdl_kin.forward(q)
        # q_ik = self.kdl_kin.inverse(pose, q+0.3) # inverse kinematics
        # if q_ik is not None:
        #     pose_sol = self.kdl_kin.forward(q_ik) # should equal pose
        # J = self.kdl_kin.jacobian(q)
        # rospy.logwarn('q:'+str(q))
        # rospy.logwarn('q_ik:'+str(q_ik))
        # rospy.logwarn('pose:'+str(pose))
        # if q_ik is not None:
        #     rospy.logwarn('pose_sol:'+str(pose_sol))
        # rospy.logwarn('J:'+str(J))

        ### START LOOP ###
        while not rospy.is_shutdown():
            self.update()
            self.publish_status()
            if self.driver_status == 'FOLLOW':
                self.update_follow()
            # Sleep between commands to robot
            self.run_rate.sleep()

        # Finish
        rospy.logwarn('SIMPLE UR - Simulation Finished')

    def update(self):
        if not self.driver_status == 'DISCONNECTED':

            # Calculate Joint Positions for "TARGET FRAME"
            try:
                F_target_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/target_frame',rospy.Time(0)))
                F_target_base = tf_c.fromTf(self.listener_.lookupTransform('/base_link','/target_frame',rospy.Time(0)))
                F_base_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/base_link',rospy.Time(0)))
                F_command = F_base_world.Inverse()*F_target_world

                M_command = tf_c.toMatrix(F_command)

                # M_current = self.kdl_kin.forward(self.q)
                q_ik = self.kdl_kin.inverse(M_command, self.q) # inverse kinematics
                if q_ik is not None:
                    pose_sol = self.kdl_kin.forward(q_ik) # should equal pose
                    # Update Joint Angles
                    # rospy.logwarn(q_ik)

                    self.q = q_ik
                    self.current_joint_positions = self.q
                    msg = JointState()
                    msg.header.stamp = rospy.get_rostime()
                    msg.header.frame_id = "robot_secondary_interface_data"
                    msg.name = self.JOINT_NAMES
                    msg.position = self.current_joint_positions
                    msg.velocity = [0]*6
                    msg.effort = [0]*6
                    self.joint_state_publisher.publish(msg)
                    
                    F = self.F_start
                    self.current_tcp_pose = tf_c.toMsg(F)
                    self.current_tcp_frame = F
                    self.broadcaster_.sendTransform(tuple(F.p),tuple(F.M.GetQuaternion()),rospy.Time.now(), '/endpoint','/base_link')
                else:
                    rospy.logwarn('no solution found')

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(str(e))

            # Get Joint Positions

            # try:
            #   F_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/endpoint',rospy.Time(0)))
            #   rospy.logwarn(F_world.p)
            #   rospy.logwarn(F_world.M.GetRPY())
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            #     rospy.logwarn(str(e))

    def set_teach_mode_call(self,req):
        if self.driver_status == 'SERVO':
            rospy.logwarn('SIMPLE UR -- cannot enter teach mode, servo mode is active')
            return 'FAILED - servo mode is active'
        else:
            if req.enable == True:
                self.rob.set_freedrive(True)
                self.driver_status = 'TEACH'
                return 'SUCCESS - teach mode enabled'
            else:
                self.rob.set_freedrive(False)
                self.driver_status = 'IDLE'
                return 'SUCCESS - teach mode disabled'

    def set_servo_mode_call(self,req):
        if self.driver_status == 'TEACH':
            rospy.logwarn('SIMPLE UR -- cannot enter servo mode, teach mode is active')
            return 'FAILED - teach mode is active'
        else:
            rospy.logwarn('MODE IS ['+req.mode+']')
            if req.mode == 'SERVO':
                self.driver_status = 'SERVO'
                return 'SUCCESS - servo mode enabled'
            elif req.mode == 'FOLLOW':
                rospy.logwarn('requested follow mode')
                if self.start_follow():
                    self.driver_status = 'FOLLOW'
                    self.reset_follow_goal()
                    return 'SUCCESS - follow mode enabled'
                else:
                    return 'FAILED - Could not create follow socket'
            elif req.mode == 'DISABLE':
                if self.driver_status == 'FOLLOW':
                    self.stop_follow()
                    self.reset_follow_goal()
                self.driver_status = 'IDLE'
                return 'SUCCESS - servo mode disabled'

    def set_stop_call(self,req):
        rospy.logwarn('SIMPLE UR - STOPPING ROBOT')
        self.rob.stop()
        return 'SUCCESS - stopped robot'

    def servo_to_pose_call(self,req): 
        if self.driver_status == 'SERVO':
            T = tf_c.fromMsg(req.target)
            a,axis = T.M.GetRotAngle()
            pose = list(T.p) + [a*axis[0],a*axis[1],a*axis[2]]
            # Check acceleration and velocity limits
            if req.accel > self.MAX_ACC:
                acceleration = self.MAX_ACC
            else:
                acceleration = req.accel
            if req.vel > self.MAX_VEL:
                velocity = self.MAX_VEL
            else:
                velocity = req.vel
            # Send command
            self.rob.movel(pose,acc=acceleration,vel=velocity)
            return 'SUCCESS - moved to pose'
        else:
            rospy.logerr('SIMPLE UR -- Not in servo mode')
            return 'FAILED - not in servo mode'

    def publish_status(self):
        self.driver_status_publisher.publish(String(self.driver_status))
        self.robot_state_publisher.publish(String(self.robot_state))

    def check_robot_state(self):
        self.robot_state == 'RUNNING SIMULATION'
        self.driver_status = 'IDLE - SIMULATION'

if __name__ == "__main__":
    robot_driver = URDriver()