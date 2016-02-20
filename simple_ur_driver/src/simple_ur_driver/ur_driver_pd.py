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
from trajectory_msgs.msg import JointTrajectoryPoint
from predicator_msgs.msg import *
from std_msgs.msg import *
import time
import threading
import socket
# URX Universal Robot Driver
import urx
# OTHER
import logging
from threading import Thread
import numpy as np
# CONTROL
from pid import PID
# ROBOT KINEMATICS
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics

''' ========================== UTILITIES =========================='''
'''
These helper functions are to make sure the robot stops when it is supposed to.
'''
def check_zero(v):
  for vv in v:
    if abs(vv) > .0001:
      return False
  return True 


def check_stop(V):
  for v in V:
    if abs(v) < .0001:
      v = 0
  return V 

def add_vectors(a,b):
  v = []
  for aa,bb in zip(a,b):
    v.append(aa + bb)
  return v
''' ========================== END UTILITIES =========================='''

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
    
    # PROG FILE
    PID_PROG = '''######################################################################
def pidProg():
  textmsg("Velocity Follow Program Started")
  MSG_QUIT = 3
  MSG_TEST = 4
  MSG_SETPOINT = 5
  Socket_Closed = True

  ## SET UP ####################################################################
  xd = [0,0,0,0,0,0]
  thread move():
    while True:
      speedj(xd, 1.0, 0)
    end
  end

  thrd = run move()

  ## MAIN LOOP #################################################################
  while (True):
    ### OPEN SOCKET ###
    if (Socket_Closed == True):
      # Keep Checking socket to see if opening it failed
      r = socket_open("192.168.1.5", 30001)
      if r == True:
        global Socket_Closed = False 
      else:
        textmsg("Socket Failed to Open")
      end
    end

    ### READ DATA ###
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
      xd = [data[1],data[2],data[3],data[4],data[5],data[6]]
    end

  end
  
  ### FINISH ###
  kill thrd
  socket_close()
  textmsg("Finished")

end
pidProg()
'''

    def __init__(self):
        rospy.init_node('ur_driver',anonymous=True)
        rospy.logwarn('SIMPLE_UR DRIVER LOADING')
        # Set State First
        self.robot_state = 'POWER OFF'
        self.driver_status = 'IDLE'
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
        self.set_point_publisher = rospy.Publisher('joint_states_goal',JointState)
        self.follow_pose_subscriber = rospy.Subscriber('/ur_robot/follow_goal',PoseStamped,self.follow_goal_cb)
        self.follow_pose_subscriber = rospy.Subscriber('/ur_robot/follow_joint_goal',JointTrajectoryPoint,self.follow_joint_goal_cb)
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
        robot = URDF.from_parameter_server()
        self.rob = urx.Robot("192.168.1.155", logLevel=logging.INFO)
        self.kdl_kin = KDLKinematics(robot, 'base_link', 'ee_link')
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

        ### Set up individual PIDs with gains ###
        self.pid = []
        self.pid.append(PID(1,0,.01))
        self.pid.append(PID(1,0,.025))
        self.pid.append(PID(1,0,.05))
        self.pid.append(PID(1,0,.1))
        self.pid.append(PID(1,0,.1))
        self.pid.append(PID(1,0,.1))

        self.current_joint_positions = self.rob.getj()
        self.set_point = self.current_joint_positions
        for i in range(6):
            self.pid[i].setPoint(self.set_point[i])

        ### START LOOP ###
        print 'Starting main loop...'
        self.update_thread = Thread(target=self.update_thread,args=())
        self.update_thread.start()
        print 'Updating control...'
        while not rospy.is_shutdown():
            #self.follow_goal_update()
            self.run_rate.sleep()
        self.update_thread.join()

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

    def update_thread(self):
        rospy.loginfo("Started main update loop successfully!")
        while not rospy.is_shutdown():
            # msg = struct.pack("!i", 10000)
            # self.rt_socket.send(msg)

            self.update()
            self.check_driver_status()
            self.check_robot_state()
            self.publish_status()
            if  self.driver_status == 'FOLLOW':
                self.update_follow()
	    else:
	        self.set_point = self.current_joint_positions
                for i in range(6):
                    self.pid[i].setPoint(self.set_point[i])
            # Sleep between commands to robot
            self.run_rate.sleep()

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
            
            # create a message about current set point if it exists
            msg = JointState()
            msg.header.stamp = rospy.get_rostime()
            msg.header.frame_id = "user_set_point"
            msg.name = self.JOINT_NAMES
            msg.position = self.set_point
            msg.velocity = [0]*6
            msg.effort = [0]*6
            self.set_point_publisher.publish(msg)

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
        print "SERVO TO POINT ({},{},{},{},{},{})".format(*self.set_point)
        current_pose = self.rob.getj()
        
        command = [0,0,0,0,0,0]
        for i in range(6):
            command[i] = self.pid[i].update(current_pose[i])

            # command[2] = pid[2].update(current_pose[2])

        command_to_send = check_stop(command)
        print command_to_send

        self.follow_sock_handle.send("({},{},{},{},{},{})".format(*command_to_send))
        self.run_rate.sleep()
        self.stopped = check_zero(command_to_send)

        #if self.follow_goal_axis_angle != None:
        #    if self.follow_sock_handle != None:
        #        self.follow_sock_handle.send("({},{},{},{},{},{})".format(*self.follow_goal_axis_angle))

    def start_follow(self):
        rospy.logwarn('Starting follow mode')
        self.follow_host = "192.168.1.5"      # The remote host
        self.follow_port = 30001                # The same port as used by the server
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
            try:
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
            except socket.error, msg:
		    rospy.loginfo(msg)
            self.follow_socket = None
            self.follow_sock_handle = None
        else:
            rospy.logwarn("Handle Not Found")

    '''
    follow_joint_goal_cb
    really simple: set PID control to joint position
    '''
    def follow_joint_goal_cb(self,msg):
        if self.driver_status == 'FOLLOW':
          print msg
          self.set_point = msg.positions
          for i in range(6):
	        self.pid[i].setPoint(self.set_point[i])
        else:
            rospy.logerr("FOLLOW NOT ENABLED!")



    '''
    follow_goal_cb
    Checks to find position of interactive marker relative to the robot.
    Solves the inverse kinematics via KDL and sets a new PID control point.
    '''
    def follow_goal_cb(self,msg):
        if True: #self.driver_status == 'FOLLOW':
           try:
                #F_target_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/endpoint_interact',rospy.Time(0)))
                #F_target_base = tf_c.fromTf(self.listener_.lookupTransform('/base_link','/endpoint_interact',rospy.Time(0)))
                #F_base_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/base_link',rospy.Time(0)))
                #F_ee_endpoint = tf_c.fromTf(self.listener_.lookupTransform('/ee_link','/endpoint',rospy.Time(0)))
                
                #self.F_command = F_base_world.Inverse()*F_target_world*F_ee_endpoint.Inverse()
		self.F_command = tf_c.fromMsg(msg.pose)
		#print msg

                self.broadcaster_.sendTransform(tuple(self.F_command.p),tuple(self.F_command.M.GetQuaternion()),rospy.Time.now(), '/goal','/base_link')

                M_command = tf_c.toMatrix(self.F_command)

                joint_positions = self.kdl_kin.inverse(M_command, self.current_joint_positions) # inverse kinematics
                if joint_positions is not None:
                    pose_sol = self.kdl_kin.forward(joint_positions) # should equal pose
                    self.set_point = joint_positions
                    for i in range(6):
                        self.pid[i].setPoint(self.set_point[i])
                else:
                    rospy.logwarn('no solution found')


           except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
               rospy.logwarn(str(e))

            #for i in range(6):
            #    self.pid[i].setPoint(self.rob.getj()[i]);
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
    try:
        robot_driver = URDriver()
    except rospy.ROSInterruptException():
        pass
