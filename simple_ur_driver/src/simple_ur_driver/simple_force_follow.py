#!/usr/bin/env python
# ROS IMPORTS
import roslib; roslib.load_manifest('simple_ur_driver')
import rospy
import tf; import tf_conversions as tf_c
import PyKDL
# ROBOT KINEMATICS
from copy import deepcopy
from robotiq_force_torque_sensor.msg import *
import robotiq_force_torque_sensor.srv as ft_srv

class Force():
    def __init__(self):
        rospy.init_node('ur_force_control',anonymous=True)
        # Set State First
        # TF
        self.broadcaster_ = tf.TransformBroadcaster()
        self.listener_ = tf.TransformListener()
        # SERVICES

        self.force_sub = rospy.Subscriber('/robotiq_force_torque_sensor',ft_sensor,self.force_cb)
        # Rate
        self.run_rate = rospy.Rate(30)

        self.F_endpoint_start = PyKDL.Frame()
        self.F_endpoint_target = PyKDL.Frame()

        self.initialized = False



        self.zero_sensor()

        while not rospy.is_shutdown():
            if not self.initialized:
                try:
                    self.F_endpoint_start = tf_c.fromTf(self.listener_.lookupTransform('/world','/ee_link',rospy.Time(0)))
                    self.F_endpoint_target = deepcopy(self.F_endpoint_start)
                    self.initialized = True
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn(str(e))

            self.run_rate.sleep()
        
    def force_cb(self,msg):
        F = [msg.Fx, msg.Fy, msg.Fz]
        filtered_F = [0,0,0]
        for i in range(3):
            if abs(F[i]) > 4:
                filtered_F[i] = F[i]
        p = self.F_endpoint_target.p
        p = PyKDL.Vector(p.x() - filtered_F[0]*.00005 , p.y()- filtered_F[2]*.00005, p.z()- filtered_F[1]*.00005)
        
        M = [msg.Mx, msg.My, msg.Mz]
        filtered_M = [0,0,0]
        for i in range(3):
            if abs(M[i]) > .2:
                filtered_M[i] = M[i]

        R = self.F_endpoint_target.M.GetRPY()
        M = PyKDL.Rotation.RPY(R[0] + filtered_M[2]*.0035 , R[1] - filtered_M[0]*.0035, R[2] - filtered_M[1]*.0035)

        

        self.F_endpoint_target.p = p
        self.F_endpoint_target.M = M
        self.broadcaster_.sendTransform(tuple(self.F_endpoint_target.p),tuple(self.F_endpoint_target.M.GetQuaternion()),rospy.Time.now(), '/endpoint_interact','/world')


    def zero_sensor(self):
        try:
            rospy.wait_for_service('/robotiq_force_torque_sensor_acc',2)
        except rospy.ROSException as e:
            rospy.logwarn('Could not find force sensor zero service')
            return

        try:
            zero_sensor_proxy = rospy.ServiceProxy('/robotiq_force_torque_sensor_acc',ft_srv.sensor_accessor)
            msg = ft_srv.sensor_accessorRequest()
            msg.command = "SET ZRO"
            result = zero_sensor_proxy(msg)
            rospy.logwarn(result)
        except rospy.ServiceException, e:
            rospy.logwarn(e)


if __name__ == '__main__':
    test_widget = Force()
