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
PROG = '''######################################################################
def pidProg():
  textmsg("PID Follow Program Started")
  MSG_QUIT = 3
  MSG_TEST = 4
  MSG_SETPOINT = 5
  Socket_Closed = True

  ## SET UP PD #################################################################
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

  ## MAIN LOOP #################################################################
  while (True):
    ### OPEN SOCKET ###
    if (Socket_Closed == True):
      # Keep Checking socket to see if opening it failed
      r = socket_open("192.168.1.5", 30000)
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
      # textmsg(data)
      qdes = [data[1],data[2],data[3],data[4],data[5],data[6]]
    end

    ### UPDATE PD ###
    qcurr = get_actual_joint_positions()
    textmsg(qcurr)
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
  
  ### FINISH ###
  kill thrd
  socket_close()
  textmsg("Finished")

end
pidProg()
'''

rospy.init_node('pid_test',anonymous=True)
run_rate = rospy.Rate(30)

### Set Up Robot ###
rob = urx.Robot("192.168.1.155", logLevel=logging.INFO)
init_pose = pose_to_send = rob.getj()
print pose_to_send
rob.send_program(PROG)
rospy.sleep(1)

follow_host = "192.168.1.5"      # The remote host
follow_port = 30000                # The same port as used by the server
try:
    rospy.logwarn('Trying to Create Socket')
    follow_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
    follow_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    follow_socket.bind((follow_host, follow_port))
    follow_socket.listen(5)
    follow_sock_handle, addr = follow_socket.accept()
    rospy.sleep(.25)
    # follow_sock_handle.send("(4)")
    rospy.logwarn('Socket Set Up')
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


rospy.logwarn('Done')

rob.cleanup()
rob.shutdown()
