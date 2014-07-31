#!/usr/bin/env python
# ROS IMPORTS
import roslib; roslib.load_manifest('simple_ur_driver')
import rospy
import tf; import tf_conversions as tf_c
import PyKDL
# MSGS and SERVICES
from simple_ur_msgs.srv import *
from sensor_msgs.msg import JointState
from std_msgs.msg import *

# URX Universal Robot Driver
import urx
# OTHER
import logging

class URDriver():

    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    def __init__(self):
        rospy.init_node('ur_driver',anonymous=True)
        rospy.logwarn('SIMPLE_UR DRIVER LOADING')
        # TF
        self.broadcaster_ = tf.TransformBroadcaster()
        self.listener_ = tf.TransformListener()
        # SERVICES
        self.servo_to_pose_service = rospy.Service('simple_ur_msgs/ServoToPose', ServoToPose, self.servo_to_pose_call)
        self.set_stop_service = rospy.Service('simple_ur_msgs/SetStop', SetStop, self.set_stop_call)
        self.set_teach_mode_service = rospy.Service('simple_ur_msgs/SetTeachMode', SetTeachMode, self.set_teach_mode_call)
        self.set_servo_mode_service = rospy.Service('simple_ur_msgs/SetServoMode', SetServoMode, self.set_servo_mode_call)
        # PUBLISHERS AND SUBSCRIBERS
        self.state_publisher = rospy.Publisher('ur_robot/state',String)
        self.joint_state_publisher = rospy.Publisher('joint_states',JointState)

        ### Set Up Robot ###
        self.rob = urx.Robot("192.168.1.155", logLevel=logging.INFO)        
        if not self.rob:
            rospy.logwarn('SIMPLE UR  - ROBOT NOT CONNECTED')
            self.state = 'DISCONNECTED'
        else:
            rospy.logwarn('SIMPLE UR - ROBOT CONNECTED SUCCESSFULLY')
            self.state = 'IDLE'
            self.rob.set_tcp((0,0,0,0,0,0))
            self.rob.set_payload(1.5, (0,0,0))

        while not rospy.is_shutdown():
            self.update()
            self.check_state()
            rospy.sleep(.01)

        # Finish
        rospy.logwarn('SIMPLE UR - ROBOT INTERFACE CLOSING')
        self.rob.shutdown()

    def update(self):
        if not self.state == 'DISCONNECTED':
            # Get Joint Positions
            self.current_joint_positions = self.rob.getj()
            msg = JointState()
            msg.header.stamp = rospy.get_rostime()
            msg.header.frame_id = "robot_secondary_interface_data"
            msg.name = self.JOINT_NAMES
            msg.position = self.current_joint_positions
            msg.velocity = [0]*6
            msg.effort = [0]*6
            self.joint_state_publisher.publish(msg)
            
            # Get TCP Position
            tcp_angle_axis = self.rob.getl()
            # Create Frame from XYZ and Angle Axis
            T = PyKDL.Frame()   
            axis = PyKDL.Vector(tcp_angle_axis[3],tcp_angle_axis[4],tcp_angle_axis[5])
            # Get norm and normalized axis
            angle = axis.Normalize()
            # Make frame
            T.p = PyKDL.Vector(tcp_angle_axis[0],tcp_angle_axis[1],tcp_angle_axis[2])
            T.M = PyKDL.Rotation.Rot(axis,angle)
            # Create Pose
            self.current_tcp_pose = tf_c.toMsg(T)
            self.current_tcp_frame = T
            self.broadcaster_.sendTransform(tuple(T.p),tuple(T.M.GetQuaternion()),rospy.Time.now(), '/endpoint','/base_link')

    def check_state(self):
        if self.state == 'DISCONNECTED':
            # TODO check for connection
            pass
        elif self.state == 'IDLE': 
            pass
        elif self.state == 'SERVO': 
            pass
        elif self.state == 'TEACH': 
            pass

    def set_teach_mode_call(self,req):
        if self.state == 'SERVO':
            return 'FAILED - servo mode is active'
        else:
            if req.enable == True:
                self.rob.set_freedrive(True)
                self.state = 'TEACH'
                return 'SUCCESS - teach mode enabled'
            else:
                self.rob.set_freedrive(False)
                self.state = 'IDLE'
                return 'SUCCESS - teach mode disabled'

    def set_servo_mode_call(self,req):
        if self.state == 'TEACH':
            return 'FAILED - teach mode is active'
        else:
            if req.enable == True:
                self.state = 'SERVO'
                return 'SUCCESS - servo mode enabled'
            else:
                self.state = 'IDLE'
                return 'SUCCESS - teach mode disabled'

    def set_stop_call(self,req):
        rospy.logwarn('SIMPLE UR - STOPPING ROBOT')
        self.rob.stop()
        return 'SUCCESS - stopped robot'

    def servo_to_pose_call(self,req):
        pass



if __name__ == "__main__":
    robot_driver = URDriver()

    # try:
    #     l = 0.05
    #     v = 0.05
    #     a = 0.3
    #     pose = rob.getl()
    #     print("robot tcp is at: ", pose)
    #     print("absolute move in base coordinate ")
    #     pose[2] += l
    #     rob.movel(pose, acc=a, vel=v)
    #     print("relative move in base coordinate ")
    #     rob.translate((0, 0, -l), acc=a, vel=v)
    #     print("relative move back and forth in tool coordinate")
    #     rob.translate_tool((0, 0, -l), acc=a, vel=v)
    #     rob.translate_tool((0, 0, l), acc=a, vel=v)
    # finally:
    #     rob.cleanup()

