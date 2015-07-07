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
    
    PID_PROG = '''def jointPD():
#JOINTPD
# The function listens for position commands to control the joint angles using PD control
  #ADDRESS = "${HOSTNAME}"
  #PORT = ${HOSTPORT}
  Socket_Closed = True
  #socket_open("192.168.1.5", 30000)
  #textmsg("Socket Open on Robot")

  kp = 0.35
  kd = 0.03

  tol = 0.0001 # radians
  qdes = [0,0,0,0,0,0] # desired position
  qdot = [0,0,0,0,0,0] # joint velocity
  err = [0,0,0,0,0,0] # joint error
  perr = [0,0,0,0,0,0] # previous error
  derr1 = [0,0,0,0,0,0] # joint error - previous error
  derr2 = [0,0,0,0,0,0] # derr1 from last step

  thread move():
    while True:
      speedj(qdot, 1.0, 0)
    end
  end

  thrd = run move()

  while (True):
    if (Socket_Closed == True):
      # Keep Checking socket to see if opening it failed
      r = socket_open("192.168.1.5", 30000)
      if r == True:
        global Socket_Closed = False 
      else:
        textmsg("Socket Failed to Open")
      end
    end

    val = socket_read_ascii_float(6)
    if val[0] < 6:
      if val[0] == 3:
        textmsg("Recieved Quit Command ... DONE")
        stopj(1.0)
        break
      #qdes = [0,0,0,0,0,0]    
      #stopj(1.0)
    else:
      qdes = [val[1],
             val[2],
       val[3],
       val[4],
       val[5],
       val[6]]
    end
    qcurr = get_actual_joint_positions()
    i = 0
    while i < 6:
      derr2[i] = derr1[i]      
      err[i] = qdes[i] - qcurr[i]
      derr1[i] = err[i] - perr[i]
      perr[i] = err[i]

      qdot[i] = kp*err[i] + kd*derr1[i]
      if (err[i] < tol) and (err[i] > -tol):
        qdot[i] = 0.0
      end

      i = i+1
    end
  end
  kill thrd
  socket_close()
end
'''

    def __init__(self):
        rospy.init_node('ur_driver',anonymous=True)
        rospy.logwarn('SIMPLE_UR DRIVER LOADING')
        # Set State First
        self.robot_state = 'POWER OFF'
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
        self.sound_pub = rospy.Publisher('/audri/sound/sound_player', String)
        # PREDICATOR INTERFACE
        self.pub_list = rospy.Publisher('/predicator/input', PredicateList)
        self.pub_valid = rospy.Publisher('/predicator/valid_input', ValidPredicates)
        self.exceed_notify = False
        rospy.sleep(.5)
        pval = ValidPredicates()
        pval.pheader.source = rospy.get_name()
        pval.predicates = ['soft_force_exceeded', 'hard_force_exceeded']
        pval.assignments = ['robot']
        self.pub_valid.publish(pval)

        # Rate
        self.run_rate = rospy.Rate(30)

        ### Set Up Robot ###
        self.rob = urx.Robot("192.168.1.155", logLevel=logging.INFO)
        if not self.rob:
            rospy.logwarn('SIMPLE UR  - ROBOT NOT CONNECTED')
            self.driver_status = 'DISCONNECTED'
            self.robot_state = 'POWER OFF'
        else:
            rospy.logwarn('SIMPLE UR - ROBOT CONNECTED SUCCESSFULLY')
            rospy.logwarn('SIMPLE UR - GOT REAL TIME <WRITE> INTERFACE TO ROBOT')        
            # self.rt_socket = socket.create_connection(('192.168.1.155', 30003), timeout=0.5)
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
            # msg = struct.pack("!i", 10000)
            # self.rt_socket.send(msg)

            self.update()
            self.check_driver_status()
            self.check_robot_state()
            self.publish_status()
            if self.driver_status == 'FOLLOW':
                self.update_follow()
            # Sleep between commands to robot
            self.run_rate.sleep()

        # Finish
        if self.driver_status == 'FOLLOW':
            self.stop_follow()
        rospy.logwarn('SIMPLE UR - ROBOT INTERFACE CLOSING')
        self.rob.cleanup()
        rospy.logwarn('SIMPLE UR - Robot Cleaning Up')
        self.rob.shutdown()
        rospy.logwarn('SIMPLE UR - Robot Interface Shut Down')
        # self.rt_socket.close()
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
        if self.follow_goal_axis_angle != None:
            if self.follow_sock_handle != None:
                self.follow_sock_handle.send("({},{},{},{},{},{})".format(*self.follow_goal_axis_angle))

    def start_follow(self):
        rospy.logwarn('Starting follow mode')
        self.follow_host = "192.168.1.5"      # The remote host
        self.follow_port = 30000                # The same port as used by the server
        rospy.loginfo('Sending follow program')
        # self.rob.send_program(self.PID_PROG,direct=True)
        self.rob.send_program(self.PID_PROG)
        rospy.sleep(1)
        rospy.loginfo('Creating follow socket')
        try:
            self.follow_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
            self.follow_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.follow_socket.bind((self.follow_host, self.follow_port))
            self.follow_socket.listen(5)
            self.follow_sock_handle, addr = self.follow_socket.accept()
            rospy.sleep(.25)
            # self.follow_sock_handle.send("(4)")
            return True
        except socket.error, msg:
            rospy.loginfo(msg)
            return False

    def stop_follow(self):
        rospy.logwarn('Stopping follow mode')
        if self.follow_sock_handle != None:
            rospy.loginfo('Sending Program Close Command')
            self.follow_sock_handle.send("(3)")
            rospy.sleep(.01)
            self.follow_sock_handle.send("(3)")
            rospy.sleep(.01)
            self.follow_sock_handle.send("(3)")
            rospy.sleep(.01)
            self.follow_sock_handle.send("(3)")
            rospy.sleep(.01)
            rospy.loginfo('Cleaning Up Follow Sockets')
            self.follow_sock_handle.close()
            self.follow_socket.close()
            self.follow_socket = None
            self.follow_sock_handle = None
        else:
            rospy.logwarn("Handle Not Found")

    def follow_goal_cb(self,msg):
        if self.driver_status == 'FOLLOW':
            # Set follow goal pose as axis-angle
            F_goal = tf_c.fromMsg(msg.pose)
            a,axis = F_goal.M.GetRotAngle()
            with self.pid_lock:
                self.follow_goal_axis_angle = list(F_goal.p) + [a*axis[0],a*axis[1],a*axis[2]]
            # Broadcast goal for debugging purposes
            self.broadcaster_.sendTransform(tuple(F_goal.p),tuple(F_goal.M.GetQuaternion()),rospy.Time.now(), '/ur_goal','/base_link')
            # Set goal pose as PID set point
            # with self.pid_lock:
            #     for pid, g in zip(self._pid, self.follow_goal_axis_angle):
            #         pid.setPoint(g)
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


        # Check Force
        F = self.rob.get_tcp_force()
        val_soft = False
        val_hard = False
        # print F[0]
        
        for f in F:
          if abs(f) >= 36 and abs(f) < 65:
            rospy.logwarn('Soft Force Exceed: [' +str(f)+']')
            if self.exceed_notify == False:
              self.sound_pub.publish(String("ping_2"))
              self.exceed_notify = True
            val_soft = True
            break
          elif abs(f) >= 65:
            rospy.logwarn('Hard Force Exceed: [' +str(f)+']')
            val_hard = True
            break
          else:
            self.exceed_notify = False

        ps = PredicateList()
        ps.pheader.source = rospy.get_name()
        ps.statements = []

        if val_soft:
          statement = PredicateStatement( predicate='soft_force_exceeded',
                                          confidence=1,
                                          value=True,
                                          num_params=1,
                                          params=['robot', '', ''])
          ps.statements += [statement]
        if val_hard:
          statement = PredicateStatement( predicate='hard_force_exceeded',
                                          confidence=1,
                                          value=True,
                                          num_params=1,
                                          params=['robot', '', ''])
          ps.statements += [statement]
        self.pub_list.publish(ps)

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
            if mode['robotMode'] == 0:
                if all([self.driver_status != 'SERVO',self.driver_status != 'FOLLOW',self.driver_status != 'TEACH']):
                    self.robot_state = 'RUNNING IDLE'
                    self.driver_status = 'IDLE'


if __name__ == "__main__":
    robot_driver = URDriver()