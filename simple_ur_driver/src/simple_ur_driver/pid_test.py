#!/usr/bin/env python
# ROS IMPORTS
import rospy
# MSGS and SERVICES
import time
import threading,logging
import socket
# URX Universal Robot Driver
import urx

# PROG FILE
PROG = '''def pidProg():
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
          limit_vel[i] = saved_vel[i] - max_vel_diff[i]
        end
        if vel_diff < -max_vel_diff[i]:
          limit_vel[i] = saved_vel[i] + max_vel_diff[i]
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
      speedl(limit_vel,1.0,.008)
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

rospy.init_node('pid_test',anonymous=True)
run_rate = rospy.Rate(30)

### Set Up Robot ###
rob = urx.Robot("192.168.1.155", logLevel=logging.INFO)
init_pose = pose_to_send = rob.getl()
print pose_to_send
rob.send_program(PROG)
rospy.sleep(1)

follow_host = "192.168.1.5"      # The remote host
follow_port = 30000                # The same port as used by the server
try:
    follow_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
    follow_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    follow_socket.bind((follow_host, follow_port))
    follow_socket.listen(5)
    follow_sock_handle, addr = follow_socket.accept()
    rospy.sleep(.25)
    follow_sock_handle.send("(4)")
except socket.error, msg:
    rospy.loginfo(msg)


# Send Program to robot

for i in range(60):
    pose_to_send[2]-=.005
    follow_sock_handle.send("({},{},{},{},{},{})".format(*pose_to_send))
    run_rate.sleep()
for i in range(60):
    pose_to_send[2]+=.005
    follow_sock_handle.send("({},{},{},{},{},{})".format(*pose_to_send))
    run_rate.sleep()
for i in range(60):
    pose_to_send[2]-=.005
    follow_sock_handle.send("({},{},{},{},{},{})".format(*pose_to_send))
    run_rate.sleep()
for i in range(60):
    pose_to_send[2]+=.005
    follow_sock_handle.send("({},{},{},{},{},{})".format(*pose_to_send))
    run_rate.sleep()

rob.cleanup()
rob.shutdown()
