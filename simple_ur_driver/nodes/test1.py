#!/usr/bin/env python
import socket
import time

prog = '''def pidProg():
  MSG_QUIT = 3
  MSG_TEST = 4
  MSG_SETPOINT = 5
  Socket_Closed=True
  while (True):
    if (Socket_Closed == True):
      socket_open("192.168.1.101", 30000)
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
    else:
      textmsg("Got a Bad Packet")  
    end

    sleep(.008)
  end
end
pidProg()
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
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)         # Create a socket object
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((HOST, PORT))        # Bind to the port
s.listen(5)                 # Now wait for client connection.
c, addr = s.accept()        # Establish connection with client.
   
import rospy

i = 0
while i < 10:
    c.send("(0.1,0.4,0.4,0.01,3.14,0.01)");
    time.sleep(.01)
    i = i + 1
c.send("(4)");
time.sleep(.01)
c.send("(6.0,7.0,8)");
time.sleep(.01)
c.send("(3)");
time.sleep(.01)
c.close()
s.close()
print "Program finish"

