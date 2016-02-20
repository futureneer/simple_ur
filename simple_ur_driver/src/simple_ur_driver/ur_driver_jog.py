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
    
    PID_PROG = '''def pidProg():
  textmsg("PID Follow Program Started")
  MSG_QUIT = 3
  MSG_TEST = 4
  MSG_SETPOINT = 5
  Socket_Closed = True

  ### PID VALUES ###
  Kp = [2.0,2.0,2.0,5.0,5.0,5.0]
  Ki = [0.0,0.0,0.0,0.0,0.0,0.0]
  Kd = [0.0,0.0,0.0,0.0,0.0,0.0]
  p_val = [0,0,0,0,0,0]
  i_val = [0,0,0,0,0,0]
  d_val = [0,0,0,0,0,0]

  pid_deriv = [0.0,0.0,0.0,0.0,0.0,0.0]
  pid_integ = [0.0,0.0,0.0,0.0,0.0,0.0]
  pid_integ_max = 500
  pid_integ_min = -500
  set_point = [0.0,0.0,0.0,0.0,0.0,0.0]
  set_pose = p[0.0,0.0,0.0,0.0,0.0,0.0]
  current_point = [0.0,0.0,0.0,0.0,0.0,0.0]
  current_pose = p[0.0,0.0,0.0,0.0,0.0,0.0]
  cmd_vel = [0.0,0.0,0.0,0.0,0.0,0.0]
  limit_vel = [0.0,0.0,0.0,0.0,0.0,0.0]
  saved_vel = [0.0,0.0,0.0,0.0,0.0,0.0]
  pid_error = [0.0,0.0,0.0,0.0,0.0,0.0]
  max_vel = .75
  max_vel_diff = [0.024,0.024,0.024,0.024,0.024,0.024]
  D = 0

  # Limit the Velocities to max_vel
  def clamp_velocities():
    # Impose velocity limits
    limit_vel = cmd_vel
    i = 0
    while i < 6:
      if cmd_vel[i] > max_vel:
        limit_vel[i] = max_vel
      end
      if cmd_vel[i] < -max_vel:
        limit_vel[i] = -max_vel
      end
      i = i + 1
    end    
  end

  def clamp_accelerations():
    i = 0
    vel_diff = 0
    while i < 6:
        vel_diff = saved_vel[i] - limit_vel[i]
        if vel_diff > max_vel_diff[i]:
          limit_vel[i] = saved_vel[i] - .1 * max_vel_diff[i]
        end
        if vel_diff < -max_vel_diff[i]:
          limit_vel[i] = saved_vel[i] + .1 * max_vel_diff[i]
        end
        i = i + 1
    end
    # Update saved Velocities
    saved_vel = limit_vel
  end

  # Set the PID setpoint from a packet
  def set_pid_setpoint(data):
    enter_critical
    point = 0
    while point < data[0]:
      set_point[point] = data[point+1]
      point = point + 1
    end
    set_pose = p[data[1],data[2],data[3],data[4],data[5],data[6]]
    pid_integ = [0.0,0.0,0.0,0.0,0.0,0.0]
    pid_deriv = [0.0,0.0,0.0,0.0,0.0,0.0]
    exit_critical
  end

  # Set the PID setpoint from a pose
  def set_pid_setpoint_from_pose(pose):
    enter_critical
    set_pose = pose
    point = 0
    while point < 6:
      set_point[point] = pose[point]
      point = point + 1
    end
    pid_integ = [0.0,0.0,0.0,0.0,0.0,0.0]
    pid_deriv = [0.0,0.0,0.0,0.0,0.0,0.0]
    exit_critical
  end

  # update the current pose of the robot in pose and list form
  def get_current_point():
    enter_critical
    current_pose = get_actual_tcp_pose()
    i = 0
    while i < 6:
      current_point[i] = current_pose[i]
      i = i + 1
    end
    exit_critical
  end

  ### PID UPDATE ###
  def update_pid(i):
    pid_error[i] = set_point[i] - current_point[i] 
    p_val[i] = Kp[i] * pid_error[i]
    d_val[i] = Kd[i] * ( pid_error[i] - pid_deriv[i])
    pid_deriv[i] = pid_error[i]
    pid_integ[i] = pid_integ[i] + pid_error[i]
    if pid_integ[i] > pid_integ_max:
      pid_integ[i] = pid_integ_max
    end
    if pid_integ[i] < pid_integ_min:
      pid_integ[i] = pid_integ_min
    end
    i_val[i] = pid_integ[i] * Ki[i]
    upd = p_val[i] + i_val[i] + d_val[i]
    return upd
  end

  ### PID UPDATE THREAD ###
  thread pid_update_thread():
    while True:
      get_current_point()
      D = pose_dist(set_pose,current_pose)
      if D > .001:
        enter_critical
        i = 0
        while i < 6:
          cmd_vel[i] = update_pid(i)
          i = i + 1
        end
        clamp_velocities()
        clamp_accelerations()
        exit_critical
      else:
        enter_critical
        limit_vel = [0.0,0.0,0.0,0.0,0.0,0.0]
        exit_critical
      end
      sync()
    end
  end

  thread move_thread():
    while True:
      speedl(limit_vel,1.0,.015)
    end
  end

  #### RUN ####

  # Set initial set point to robot position
  textmsg("Setting Initial PID Set Point")
  set_pid_setpoint_from_pose( get_actual_tcp_pose() )
  textmsg(set_point)
  thread_pid_h = run pid_update_thread()
  thread_move_h = run move_thread()

  ## MAIN LOOP
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
      set_pid_setpoint(data)

    # else:
      # textmsg("Got a Bad Packet")  
    end

    sleep(.1)
  end
  # When finished kill pid thread
  kill thread_pid_h
  kill thread_move_h
  textmsg("Finished PID Thread")
end
pidProg()
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
        self.jog_pose_subscriber = rospy.Subscriber('/ur_robot/jog',JointState,self.jog)
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
        self.rob.stopl()
        return 'SUCCESS - stopped robot'

    def jog(self,msg):
        # New Code
        V = msg.velocity
        # rospy.logwarn(V)
        if all(a == 0.0 for a in V):
            vel_cmd = [0,0,0,0,0,0]
        else:
            F_endpoint = self.current_tcp_frame
            F_VEL = PyKDL.FrameVel(PyKDL.Frame(),PyKDL.Twist(PyKDL.Vector(V[0],V[1],V[2]),PyKDL.Vector(V[3],V[4],V[5])))
            F_VEL_MOD = F_endpoint*F_VEL

            t = F_VEL_MOD.GetTwist()
            vel_cmd = [t[0],t[1],t[2],t[3],t[4],t[5]]
            # F_vels = PyKDL.Frame()
            # F_vels.p = PyKDL.Vector(vels[3],vels[4],vels[5])
            # F_vels.M = PyKDL.Rotation.RPY(vels[0],vels[1],vels[2])
            # F_cmd = F_endpoint*F_vels
            # Fxyz = F_cmd.p
            # Frpy = F_cmd.M.GetRPY()
            # vel_cmd = [Fxyz.x(),Fxyz.y(),Fxyz.z(),Frpy[0],Frpy[1],Frpy[2]]
            # # vel_cmd = V

        ###########
        # rospy.logwarn("GOT JOG MSG")
        rospy.logwarn(vel_cmd)
        if self.driver_status == 'IDLE': 
            # self.rob.speedl(msg.velocity,self.MAX_ACC,10)

            self.rob.speedl(vel_cmd,self.MAX_ACC,10)
            pass

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


        # # Check Force
        # F = self.rob.get_tcp_force()
        # val_soft = False
        # val_hard = False
        # # print F[0]
        
        # for f in F:
        #   if abs(f) >= 36 and abs(f) < 65:
        #     rospy.logwarn('Soft Force Exceed: [' +str(f)+']')
        #     if self.exceed_notify == False:
        #       self.sound_pub.publish(String("ping_2"))
        #       self.exceed_notify = True
        #     val_soft = True
        #     break
        #   elif abs(f) >= 65:
        #     rospy.logwarn('Hard Force Exceed: [' +str(f)+']')
        #     val_hard = True
        #     break
        #   else:
        #     self.exceed_notify = False

        # ps = PredicateList()
        # ps.pheader.source = rospy.get_name()
        # ps.statements = []

        # if val_soft:
        #   statement = PredicateStatement( predicate='soft_force_exceeded',
        #                                   confidence=1,
        #                                   value=True,
        #                                   num_params=1,
        #                                   params=['robot', '', ''])
        #   ps.statements += [statement]
        # if val_hard:
        #   statement = PredicateStatement( predicate='hard_force_exceeded',
        #                                   confidence=1,
        #                                   value=True,
        #                                   num_params=1,
        #                                   params=['robot', '', ''])
        #   ps.statements += [statement]
        # self.pub_list.publish(ps)

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