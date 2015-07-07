#!/usr/bin/env python
# ROS IMPORTS
import rospy
# MSGS and SERVICES
import time
import threading,logging
import socket
# URX Universal Robot Driver
import urx
from pid import PID

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


# PROG FILE
PROG = '''######################################################################
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

rospy.init_node('pid_test',anonymous=True)
run_rate = rospy.Rate(50)

### Set Up Robot ###
rob = urx.Robot("192.168.1.155", logLevel=logging.INFO)
init_pose = current_pose = rob.getj()
print current_pose
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



pid = []
zero_command = command = [0,0,0,0,0,0]
set_point = add_vectors(init_pose, [.3,.3,.3,.3,.3,.3])

for i in range(6):
  pid.append(PID(1,0,.1))

for i in range(6):
  pid[i].setPoint(set_point[i])

# pid[2].setPoint(init_pose[2]-.1)


stopped = False

while not stopped:
# for i in range(100):
  print "SERVO TO POINT ["+str(i)+"]"
  current_pose = rob.getj()

  for i in range(6):
    command[i] = pid[i].update(current_pose[i])

  # command[2] = pid[2].update(current_pose[2])

  command_to_send = check_stop(command)
  print command_to_send

  follow_sock_handle.send("({},{},{},{},{},{})".format(*command_to_send))
  run_rate.sleep()
  stopped = check_zero(command_to_send)

follow_sock_handle.send("({},{},{},{},{},{})".format(*zero_command))
run_rate.sleep()
follow_sock_handle.send("(3)")
run_rate.sleep()
print "MOVE FINISHED"


#vels = [0,0,.5,0,0,0]


# Send Program to robot

#for i in range(100):
#    print vels
#    vels[2]-=.005
#    follow_sock_handle.send("({},{},{},{},{},{})".format(*vels))
#    run_rate.sleep()


# for i in range(60):
#     pose_to_send[2]+=.005
#     follow_sock_handle.send("({},{},{},{},{},{})".format(*pose_to_send))
#     run_rate.sleep()
# for i in range(60):
#     pose_to_send[2]-=.005
#     follow_sock_handle.send("({},{},{},{},{},{})".format(*pose_to_send))
#     run_rate.sleep()
# for i in range(60):
#     pose_to_send[2]+=.005
#     follow_sock_handle.send("({},{},{},{},{},{})".format(*pose_to_send))
#     run_rate.sleep()


rospy.logwarn('Done')

rob.cleanup()
rob.shutdown()
