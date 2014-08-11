#!/usr/bin/env python
import socket
import time

prog = '''def receive_3_coordinates_from_host():
  Move_To_Pos=p[0, 0.4, 0.4, 0, 3.14, 0]
  Pointer=0
  Receive_Data=[0, 0, 0, 0, 0, 0, 0]
  Socket_Closed=True
  while (True):
    global Move_To_Pos = Move_To_Pos
    varmsg("Move_To_Pos",Move_To_Pos)
    global Receive_Data = Receive_Data
    varmsg("Receive_Data",Receive_Data)
    global Pointer = Pointer
    varmsg("Pointer",Pointer)
    if (Socket_Closed ==  True  ):
      socket_open("192.168.1.101", 30000)
      global Socket_Closed = False 
      varmsg("Socket_Closed",Socket_Closed)
    end
    pose_position = get_forward_kin()
    joint_positions = get_joint_positions()
    socket_send_string(pose_position)
    sleep(1.0)
    socket_send_string(joint_positions)
    sleep(1.0)
    socket_send_string("Asking_Waypoint_1")
    sleep(3.0)
    Receive_Data = socket_read_ascii_float(6)
    textmsg(Receive_Data)
    global Pointer = 0
    varmsg("Pointer",Pointer)
    while (Pointer < Receive_Data[0]):
      Move_To_Pos[Pointer] = Receive_Data[Pointer+1]
      global Pointer = Pointer+1
      varmsg("Pointer",Pointer)
    end
    varmsg("Move_To_Pos", Move_To_Pos)
    # movel(Move_To_Pos)
    socket_send_string("Asking_Waypoint_2")
    sleep(3.0)
    Receive_Data = socket_read_ascii_float(6)
    textmsg(Receive_Data)
    global Pointer = 0
    varmsg("Pointer",Pointer)
    while (Pointer < Receive_Data[0]):
      Move_To_Pos[Pointer] = Receive_Data[Pointer+1]
      global Pointer = Pointer+1
      varmsg("Pointer",Pointer)
    end
    varmsg("Move_To_Pos", Move_To_Pos)
    # movel(Move_To_Pos)
    socket_send_string("Asking_Waypoint_3")
    sleep(3.0)
    Receive_Data = socket_read_ascii_float(6)
    textmsg(Receive_Data)
    global Pointer = 0
    varmsg("Pointer",Pointer)
    while (Pointer < Receive_Data[0]):
      Move_To_Pos[Pointer] = Receive_Data[Pointer+1]
      global Pointer = Pointer+1
      varmsg("Pointer",Pointer)
    end
    varmsg("Move_To_Pos", Move_To_Pos)
    # movel(Move_To_Pos)
  end
end
receive_3_coordinates_from_host()
'''
sock = socket.create_connection(('192.168.1.155', 30003), timeout=0.5)
print 'created socket'
time.sleep(1)

prog.strip()
print "Sending program: " + prog
if type(prog) != bytes:
    prog = prog.encode()
sock.send(prog+b'\n')
sock.close()

print 'program loaded'

HOST = "192.168.1.101"      # The remote host
PORT = 30000                # The same port as used by the server

print "Starting Program"
s = socket.socket()         # Create a socket object
count = 0
count_2 = 0
data = 0
s.bind((HOST, PORT))        # Bind to the port
s.listen(5)                 # Now wait for client connection.
c, addr = s.accept()        # Establish connection with client.
   
while (count < 1):

    count = count + 1
    print "The count is:", count
    time.sleep(0.5)
    print ""
    msg = c.recv(1024)
    print "The robot position is(XYZ vector)"
    print msg
    print ""
    time.sleep(0.5)
    msg = c.recv(1024)
    print "The robot position is(Joint angle)"
    print msg

    while (count_2 < 3):
        count_2 = count_2 + 1
        print ""
        msg = c.recv(1024)
        print msg
        time.sleep(1)
        if msg == "Asking_Waypoint_1":
            c.send("(0.1,0.4,0.4,0.01,3.14,0.01)");
        if msg == "Asking_Waypoint_2":
            c.send("(0.1,0.4,0.3,0.01,3.14,0.01)");
        if msg == "Asking_Waypoint_3":
            c.send("(0.1,0.4,0.2,0.01,3.14,0.01)");
    print ""
    msg = c.recv(1024)
    print "The robot position is(XYZ vector)"
    print msg
    print ""
    msg = c.recv(1024)
    print "The robot position is(Joint angle)"
    print msg

c.close()

print "Program finish"

