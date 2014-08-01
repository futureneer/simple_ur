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
from robotiq_c_model_control.srv import *

class URStatusPanel(Plugin):
    def __init__(self,context):
        super(URStatusPanel,self).__init__(context)

        self.setObjectName('UR5 Status Panel')
        self._widget = QWidget()

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('simple_ur_driver') + '/ui/ur_status.ui'
        uic.loadUi(ui_path, self._widget)
        self._widget.setObjectName('UR5StatusPanel')
        self._widget.setWindowTitle('UR5 Status Panel')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # # Parameters
        self.freedrive = False
        self.servo = False
        self.status = 'DISCONNECTED'
        self.listener_ = TransformListener()

        self.status_sub = rospy.Subscriber('/ur_robot/state',String,self.status_cb)

        self._widget.freedrive_enable_btn.clicked.connect(self.freedrive_enable)
        self._widget.freedrive_disable_btn.clicked.connect(self.freedrive_disable)
        self._widget.servo_enable_btn.clicked.connect(self.servo_enable)
        self._widget.servo_disable_btn.clicked.connect(self.servo_disable)

        self._widget.gripper_open_btn.clicked.connect(self.gripper_open)
        self._widget.gripper_close_btn.clicked.connect(self.gripper_close)

    def gripper_open(self):
        try:
            rospy.wait_for_service('/robotiq_c_model_control/Open',2)
        except rospy.ROSException as e:
            print 'Could not find gripper Open service'
            self._widget.msg_label.setText("NO GRIPPER OPEN SERVICE")
            return
        try:
            gripper_open_proxy = rospy.ServiceProxy('/robotiq_c_model_control/Open',Open)
            msg = OpenRequest()
            msg.state = True
            msg.wait = True
            result = gripper_open_proxy(msg)
            self._widget.gripper_state_label.setText('OPEN')
            self._widget.gripper_state_label.setStyleSheet('color:#ffffff;background-color:#3FC4FC')
            self._widget.msg_label.setText("GRIPPER OPENED")
        except rospy.ServiceException, e:
            print e

    def gripper_close(self):
        try:
            rospy.wait_for_service('/robotiq_c_model_control/Open',2)
        except rospy.ROSException as e:
            print 'Could not find gripper Open service'
            self._widget.msg_label.setText("NO GRIPPER OPEN SERVICE")
            return
        try:
            gripper_open_proxy = rospy.ServiceProxy('/robotiq_c_model_control/Open',Open)
            msg = OpenRequest()
            msg.state = False
            msg.wait = True
            result = gripper_open_proxy(msg)
            self._widget.gripper_state_label.setText('CLOSED')
            self._widget.gripper_state_label.setStyleSheet('color:#ffffff;background-color:#6AAAC4')
            self._widget.msg_label.setText("GRIPPER CLOSED")
        except rospy.ServiceException, e:
            print e

    def status_cb(self,msg):
        self.status = msg.data

    def servo_enable(self):
        rospy.logwarn('enabling servo')
        if self.freedrive == False:
            if self.servo == False:
                try:
                    rospy.wait_for_service('/simple_ur_msgs/SetServoMode',2)
                except rospy.ROSException as e:
                    print 'Could not find SetServoMode service'
                    self._widget.msg_label.setText("NO SERVO_ENABLE SERVICE")
                    return
                try:
                    servo_mode_service = rospy.ServiceProxy('/simple_ur_msgs/SetServoMode',SetServoMode)
                    result = servo_mode_service(True)
                    self.servo = True
                    self._widget.servo_enable_label.setText('ENABLED')
                    self._widget.servo_enable_label.setStyleSheet('color:#ffffff;background-color:#ADE817')
                    self._widget.msg_label.setText("SERVO ENABLED")
                except rospy.ServiceException, e:
                    print e
            else:
                self._widget.msg_label.setText('SERVO ALREAD ENABLED')
        else:
            self._widget.msg_label.setText("CANT SERVO, FREEDRIVE ENABLED")

    def servo_disable(self):
        rospy.logwarn('disabling servo')
        if self.freedrive == False:
            if self.servo == True:
                    try:
                        rospy.wait_for_service('/simple_ur_msgs/SetServoMode',2)
                    except rospy.ROSException as e:
                        print 'Could not find SetServoMode service'
                        self._widget.msg_label.setText("NO SERVO_ENABLE SERVICE")
                        return
                    try:
                        servo_mode_service = rospy.ServiceProxy('/simple_ur_msgs/SetServoMode',SetServoMode)
                        result = servo_mode_service(False)
                        self.servo = False
                        self._widget.servo_enable_label.setText('DISABLED')
                        self._widget.servo_enable_label.setStyleSheet('color:#ffffff;background-color:#FF9100')
                        self._widget.msg_label.setText("SERVO DISABLED")
                    except rospy.ServiceException, e:
                        print e
            else:
                 self._widget.msg_label.setText("SERVO IS NOT ENABLED")  
        else:
            self._widget.msg_label.setText("CANT DISABLE SERVO, FREEDRIVE ENABLED")

    def freedrive_enable(self):
        rospy.logwarn('enabling freedrive')
        if self.servo == False:
            if self.freedrive == False:
                try:
                    rospy.wait_for_service('/simple_ur_msgs/SetTeachMode',2)
                except rospy.ROSException as e:
                    print 'Could not find freedrive service'
                    return
                try:
                    teach_mode_service = rospy.ServiceProxy('/simple_ur_msgs/SetTeachMode',SetTeachMode)
                    result = teach_mode_service(True)
                    # print 'Service returned: ' + str(result.ack)
                    self.freedrive = True
                    self._widget.freedrive_enable_label.setText('ENABLED')
                    self._widget.freedrive_enable_label.setStyleSheet('color:#ffffff;background-color:#ADE817')
                    self._widget.msg_label.setText("FREEDRIVE ENABLED")
                except rospy.ServiceException, e:
                    print e
            else:
                self._widget.msg_label.setText("FREEDRIVE IS ALREADY ENABLED")
        else:
            self._widget.msg_label.setText("CANT FREEDRIVE, SERVO ENABLED")

    def freedrive_disable(self):
        if self.freedrive == True:
            try:
                rospy.wait_for_service('/simple_ur_msgs/SetTeachMode',2)
            except rospy.ROSException as e:
                print 'Could not find freedrive service'
                return
            try:
                teach_mode_service = rospy.ServiceProxy('/simple_ur_msgs/SetTeachMode',SetTeachMode)
                result = teach_mode_service(False)
                # print 'Service returned: ' + str(result.ack)
                self.freedrive = False
                self._widget.freedrive_enable_label.setText('DISABLED')
                self._widget.freedrive_enable_label.setStyleSheet('color:#ffffff;background-color:#FF9100')
                self._widget.msg_label.setText("FREEDRIVE DISABLED")
            except rospy.ServiceException, e:
                print e
        else:
            print 'Freedrive is not enabled'

    def check_status(self):
        if self.status == 'IDLE':
            self._widget.mode_label.setText(str(self.status))
            self._widget.mode_label.setStyleSheet('color:#ffffff; background-color:#EBCF1A')
        elif self.status == 'SERVO':
            self._widget.mode_label.setText(str(self.status))
            self._widget.mode_label.setStyleSheet('color:#ffffff; background-color:#AFEB1A')
        elif self.status == 'TEACH':
            self._widget.mode_label.setText(str(self.status))
            self._widget.mode_label.setStyleSheet('color:#ffffff; background-color:#1AA5EB')
        elif self.status == 'DISCONNECTED':
            self.servo = False
            self._widget.servo_enable_label.setText('DISABLED')
            self._widget.servo_enable_label.setStyleSheet('color:#ffffff;background-color:#FF9100')
            self._widget.msg_label.setText("SERVO DISABLED")
            self._widget.mode_label.setText(str(self.status))
            self._widget.mode_label.setStyleSheet('color:#ffffff; background-color:#EB1A1D')

    # def stop_robot(self):
    #     rospy.wait_for_service('/simple_ur_msgs/stop')
    #     try:
    #         stop_service = rospy.ServiceProxy('/simple_ur_msgs/stop',stop)
    #         result = stop_service('')
    #         return result
    #     except rospy.ServiceException, e:
    #         print e

    def shutdown_plugin(self):
        # unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass





















