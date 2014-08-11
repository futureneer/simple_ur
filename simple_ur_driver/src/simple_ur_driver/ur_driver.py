#!/usr/bin/env python
# ROS IMPORTS
import roslib; roslib.load_manifest('simple_ur_driver')
import rospy
import tf; import tf_conversions as tf_c
import PyKDL
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
    MSG_QUIT = 2
    MSG_TEST = 3
    MSG_SETPOINT = 4
    MULT_jointstate = 10000.0
    MULT_time = 1000000.0
    MULT_blend = 1000.0
    
    PID_PROG = '''def pidProg():
  textmsg("PID Follow Program Started")
  MSG_QUIT = 3
  MSG_TEST = 4
  MSG_SETPOINT = 5
  Socket_Closed = True

  ## PID ##
  Kp = [10.0,10.0,10.0,20.0,20.0,20.0]
  Ki = [0.0,0.0,0.0,0.0,0.0,0.0]
  Kd = [0.0,0.0,0.0,0.0,0.0,0.0]
  P_value = [0,0,0,0,0,0]
  I_value = [0,0,0,0,0,0]
  D_value = [0,0,0,0,0,0]

  Derivator = [0.0,0.0,0.0,0.0,0.0,0.0]
  Integrator = [0.0,0.0,0.0,0.0,0.0,0.0]
  Integrator_max = 500
  Integrator_min = -500
  set_point = [0.0,0.0,0.0,0.0,0.0,0.0]
  error = [0.0,0.0,0.0,0.0,0.0,0.0]

  def set_pid_setpoint(data):
    point = 0
    while point < data[0]:
      set_point[point] = data[point+1]
      point = point + 1
    end
    Integrator = [0.0,0.0,0.0,0.0,0.0,0.0]
    Derivator = [0.0,0.0,0.0,0.0,0.0,0.0]
  end

  ## PID ##

  ## MAIN LOOP
  while (True):
    if (Socket_Closed == True):
      socket_open("192.168.1.5", 30000)
      global Socket_Closed = False 
    end

    data = socket_read_ascii_float(6)
    if data[0] == 1:
      textmsg("Got Command")
      if data[1] == MSG_QUIT:
        textmsg("Recieved Quit Command ... DONE")
        break
      elif data[1] == MSG_TEST:
        textmsg("Recieved Test Message")
      end
    elif data[0] == 6:
      textmsg(data)
      set_pid_setpoint(data)
      textmsg(set_point)

    # else:
      # textmsg("Got a Bad Packet")  
    end

    sleep(.008)
  end
end
pidProg()
'''

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
        self.driver_status_publisher = rospy.Publisher('/ur_robot/driver_status',String)
        self.robot_state_publisher = rospy.Publisher('/ur_robot/robot_state',String)
        self.joint_state_publisher = rospy.Publisher('joint_states',JointState)
        self.follow_pose_subscriber = rospy.Subscriber('/ur_robot/follow_goal',PoseStamped,self.follow_goal_cb)
        # Rate
        self.run_rate = rospy.Rate(50)

        ### Set Up Robot ###
        self.rob = urx.Robot("192.168.1.155", logLevel=logging.INFO)
        if not self.rob:
            rospy.logwarn('SIMPLE UR  - ROBOT NOT CONNECTED')
            self.driver_status = 'DISCONNECTED'
            self.robot_state = 'POWER OFF'
        else:
            rospy.logwarn('SIMPLE UR - ROBOT CONNECTED SUCCESSFULLY')
            rospy.logwarn('SIMPLE UR - GOT REAL TIME <WRITE> INTERFACE TO ROBOT')        
            self.rt_socket = socket.create_connection(('192.168.1.155', 30003), timeout=0.5)
            rospy.logwarn('SIMPLE UR - GOT REAL TIME <READ> INTERFACE TO ROBOT')
            self.rtm = self.rob.get_realtime_monitor()
            self.driver_status = 'IDLE'

        ### Set Up PID ###
        self.follow_goal_reached = True
        self.pid_lock = threading.Lock()
        self.reset_follow_goal()
        self.follow_socket = None
        self.follow_sock_handle = None

        ### START LOOP ###
        while not rospy.is_shutdown():
            msg = struct.pack("!i", 10000)
            self.rt_socket.send(msg)

            self.update()
            self.check_driver_status()
            self.check_robot_state()
            self.publish_status()
            if self.driver_status == 'FOLLOW':
                self.update_follow()
            # Sleep between commands to robot
            self.run_rate.sleep()

        # Finish
        rospy.logwarn('SIMPLE UR - ROBOT INTERFACE CLOSING')
        self.rob.cleanup()
        rospy.logwarn('SIMPLE UR - Robot Cleaning Up')
        self.rob.shutdown()
        rospy.logwarn('SIMPLE UR - Robot Interface Shut Down')
        self.rt_socket.close()
        rospy.logwarn('SIMPLE UR - Real Time <WRITE> Socket Closed')

    def reset_follow_goal(self):
        rospy.logwarn('RESET FOLLOW GOAL')
        self.follow_goal_axis_angle = self.current_axis_angle = self.rtm.get_all_data(wait=False)['tcp']
        
    def update(self):
        if not self.driver_status == 'DISCONNECTED':
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

    def update_follow(self):
        self.follow_sock_handle.send("(0.1,0.2,0.3,0.41,3.14,0.01)")
        pass
        # if not self.follow_goal_reached:
        #     vel_cmd = []
        #     # append a velocity update for each parameter to command velocities
        #     with self.pid_lock:
        #         for pid, p in zip(self._pid, self.current_axis_angle):
        #             vel_cmd.append(pid.update(p))
        #     # Check velocities against limits
        #     for v, i in zip(vel_cmd, range(6)):
        #         if v > 0:
        #             if v > self.MAX_VEL:
        #                 vel_cmd[i] = self.MAX_VEL
        #         else:
        #             if v < -self.MAX_VEL:
        #                 vel_cmd[i] = -self.MAX_VEL
        #     # Scale Acceleration
        #     vel_avg = abs(sum(vel_cmd)/6.0)
        #     acc_scale = (vel_avg/self.MAX_VEL)
        #     scaled_accel = self.MAX_ACC*acc_scale
        #     # clamp lower accel limit
        #     if scaled_accel < .05: scaled_accel = .05
        #     # Append other parameters to vel command
        #     vel_cmd.append(scaled_accel)
        #     # print 'acc: '+str(scaled_accel)+' avg vel: '+str(vel_avg)
        #     vel_cmd.append(self.follow_timeout)
        #     # Create and clean up program
        #     prog = "speedl([{},{},{},{},{},{}], a={}, t_min={})\n".format(*vel_cmd)
        #     # rospy.logwarn(prog)
        #     if type(prog) != bytes:
        #         prog = prog.encode()
        #     # Send command to socket
        #     self.rt_socket.send(prog)
        # # DEBUG    
        # # else:
        # #     pass
        #     # rospy.loginfo('Goal: <'+str(self.follow_goal_axis_angle)+'>')
        # # DEBUG

        
        # # Check to see if follow goal is reached
        # if self.reached_goal(self.follow_goal_axis_angle, self.current_axis_angle, .001):
        #     if self.follow_goal_reached == False:
        #         rospy.logwarn('SIMPLE UR - Follow Goal Reached')
        #         self.follow_goal_reached = True
        # else:
        #     self.follow_goal_reached = False

    def start_follow(self):
        rospy.loginfo('Setting up follow mode')
        self.follow_host = "192.168.1.5"      # The remote host
        self.follow_port = 30000                # The same port as used by the server
        rospy.loginfo('Sending follow program')
        self.rob.send_program(self.PID_PROG,direct=True)
        rospy.loginfo('Creating follow socket')
        try:
            self.follow_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
            self.follow_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.follow_socket.bind((self.follow_host, self.follow_port))
            self.follow_socket.listen(5)
            self.follow_sock_handle, addr = self.follow_socket.accept()
            rospy.sleep(.25)
            self.follow_sock_handle.send("(4)")
            return True
        except socket.error, msg:
            rospy.loginfo(msg)
            return False

    def stop_follow(self):
        if self.follow_sock_handle != None:
            rospy.loginfo('Sending Program Close Command')
            self.follow_sock_handle.send("(3)")
            rospy.sleep(.01)
            rospy.loginfo('Cleaning Up Follow Sockets')
            self.follow_sock_handle.close()
            self.follow_socket.close()
        else:
            rospy.logwarn("Handle Not Found")

    def follow_goal_cb(self,msg):
        if self.driver_status == 'FOLLOW':
            # Set follow goal pose as axis-angle
            F_goal = tf_c.fromMsg(msg.pose)
            a,axis = F_goal.M.GetRotAngle()
            self.follow_goal_axis_angle = list(F_goal.p) + [a*axis[0],a*axis[1],a*axis[2]]
            # Broadcast goal for debugging purposes
            self.broadcaster_.sendTransform(tuple(F_goal.p),tuple(F_goal.M.GetQuaternion()),rospy.Time.now(), '/ur_goal','/base_link')
            # Set goal pose as PID set point
            with self.pid_lock:
                for pid, g in zip(self._pid, self.follow_goal_axis_angle):
                    pid.setPoint(g)
        else:
            rospy.logerr("FOLLOW NOT ENABLED!")


    def reached_goal(self,a,b,val):
        ''' returns true if distance is SMALLER than val'''
        v1 = np.array(a)
        v2 = np.array(b)
        res = np.sum(np.abs(np.subtract(v1,v2)))
        # rospy.logwarn('DISTANCE = ['+str(res)+']')
        if res < val:
            return True
        else:
            return False

    def check_driver_status(self):
        if self.driver_status == 'DISCONNECTED':
            pass
        elif self.driver_status == 'IDLE': 
            pass
        elif self.driver_status == 'SERVO': 
            pass
        elif self.driver_status == 'FOLLOW': 
            pass
        elif self.driver_status == 'TEACH': 
            pass

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
        mode = self.rob.get_all_data()['RobotModeData']

        if not mode['isPowerOnRobot']:
            self.robot_state = 'POWER OFF'
            return

        if mode['isEmergencyStopped']:
            self.robot_state = 'E-STOPPED'
            self.driver_status = 'IDLE - WARN'
        elif mode['isSecurityStopped']:
            self.robot_state = 'SECURITY STOP'
            self.driver_status = 'IDLE - WARN'
        elif mode['isProgramRunning']:
            self.robot_state ='RUNNING PROGRAM'
        else:
            self.robot_state = 'RUNNING IDLE'


if __name__ == "__main__":
    robot_driver = URDriver()