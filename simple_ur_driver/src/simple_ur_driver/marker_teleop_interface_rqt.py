#!/usr/bin/env python
import roslib; roslib.load_manifest('simple_ur_driver')
import rospy
# QT
from qt_gui.plugin import Plugin
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
# Other
import actionlib
from std_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
import PyKDL
from simple_ur_msgs.srv import *
import tf; from tf import *
import tf_conversions as tf_c
import rospkg

class URMarkerTeleopPanel(Plugin):
    def __init__(self,context):
        super(URMarkerTeleopPanel,self).__init__(context)

        self.setObjectName('UR5 Marker Teleop Panel')
        self._widget = QWidget()

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('simple_ur_driver') + '/ui/marker_teleop.ui'
        uic.loadUi(ui_path, self._widget)
        self._widget.setObjectName('UR5MarkerTeleopPanel')
        self._widget.setWindowTitle('UR5 Marker Teleop Panel')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.status_sub = rospy.Subscriber('/ur_robot/state',String,self.status_cb)
        # Parameters
        self.status = 'DISCONNECTED'
        self.servo_enable = False
        self.listener_ = TransformListener()
        self.broadcaster_ = TransformBroadcaster()

        self._widget.servo_to_btn.clicked.connect(self.servo_to_pose)

        # if self.status == 'DISCONNECTED':
        #     rospy.logwarn('WARNING <<< DID NOT HEAR FROM ROBOT')
        #     rospy.sleep(1)
        
        rospy.logwarn('MARKER TELEOP INTERFACE READY')

    def status_cb(self,msg):
        self.status = msg.data

    def servo_to_pose(self):

        if self.status != 'SERVO':
            rospy.logwarn('ROBOT NOT IN SERVO MODE')
            return
        else:
            rospy.wait_for_service('/simple_ur_msgs/Servo')
            try:
                pose_servo_proxy = rospy.ServiceProxy('/simple_ur_msgs/ServoToPose',ServoToPose)
                
                F_target_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/target_frame',rospy.Time(0)))
                F_target_base = tf_c.fromTf(self.listener_.lookupTransform('/base_link','/target_frame',rospy.Time(0)))
                F_base_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/base_link',rospy.Time(0)))
                F_command = F_base_world.Inverse()*F_target_world
                    
                msg = simple_ur_msgs.srv.ServoToPoseRequest()
                msg.target = tf_c.toMsg(F_command)
                msg.accel = .7
                msg.vel = .3
                # Send Servo Command
                rospy.logwarn('Single Servo Move Started')
                result = pose_servo_proxy(msg)
                rospy.logwarn('Single Servo Move Finished')
                rospy.logwarn(str(result.ack))
            except rospy.ServiceException, e:
                print e

    def shutdown_plugin(self):
        # unregister all publishers here

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass