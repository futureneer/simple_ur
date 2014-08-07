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
import thread

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

        self.driver_status_sub = rospy.Subscriber('/ur_robot/driver_status',String,self.driver_status_cb)
        self.target_pub = rospy.Publisher('/ur_robot/follow_goal',PoseStamped)

        # Parameters
        self.driver_status = 'DISCONNECTED'
        self.follow = False
        self.listener_ = TransformListener()
        self.broadcaster_ = TransformBroadcaster()

        self._widget.servo_to_btn.clicked.connect(self.servo_to_pose)

        self._widget.follow_start_btn.clicked.connect(self.follow_start)
        self._widget.follow_stop_btn.clicked.connect(self.follow_stop)

        self.update_timer_ = QTimer(self)
        self.connect(self.update_timer_, QtCore.SIGNAL("timeout()"),self.update)
        self.update_timer_.start(100)

        rospy.logwarn('MARKER TELEOP INTERFACE READY')

    def follow_stop(self):
        self.follow = False

    def follow_start(self):
        if self.driver_status == 'FOLLOW':
            self.follow = True
        else:
            rospy.logwarn('The Driver is not in follow mode')

    def update(self):
        if self.follow == True:
            try:
                F_target_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/target_frame',rospy.Time(0)))
                F_target_base = tf_c.fromTf(self.listener_.lookupTransform('/base_link','/target_frame',rospy.Time(0)))
                F_base_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/base_link',rospy.Time(0)))
                F_command = F_base_world.Inverse()*F_target_world

                cmd = PoseStamped()
                cmd.pose = tf_c.toMsg(F_command)
                self.target_pub.publish(cmd)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(str(e))

    def status_cb(self,msg):
        self.driver_status = msg.data
        if not self.driver_status == 'FOLLOW':
            self.follow = False

    def servo_to_pose(self):
        try:
            thread.start_new_thread(self.servo_fn,('',0))
        except Exception, errtxt:
            rospy.logwarn(errtxt)

    def servo_fn(self,val,*args):
        if self.driver_status != 'SERVO':
            rospy.logwarn('ROBOT NOT IN SERVO MODE')
            return
        else:
            rospy.wait_for_service('/simple_ur_msgs/ServoToPose')
            try:
                pose_servo_proxy = rospy.ServiceProxy('/simple_ur_msgs/ServoToPose',ServoToPose)
                
                F_target_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/target_frame',rospy.Time(0)))
                F_target_base = tf_c.fromTf(self.listener_.lookupTransform('/base_link','/target_frame',rospy.Time(0)))
                F_base_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/base_link',rospy.Time(0)))
                F_command = F_base_world.Inverse()*F_target_world
                    
                msg = ServoToPoseRequest()
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
        pass
        # unregister all publishers here

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass