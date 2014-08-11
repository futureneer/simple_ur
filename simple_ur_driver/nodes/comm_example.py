#!/usr/bin/env python
import rospy
import numpy as np
from copy import deepcopy
import urx
import logging
import socket
import struct
from time import sleep

if __name__ == "__main__":

    sock = socket.create_connection(('192.168.1.155', 30003), timeout=0.5)
    print 'created socket'
    rospy.sleep(1)

    prog='''def testProg():  
  textmsg("*** Test Program Starting ***")
  MSG_TEST = 3
  textmsg("Opening RT Socket")
  socket_open("192.168.1.155", 30003)
  textmsg("Robot Communicating Properly over Realtime PORT 30003")

  # MAIN LOOP
  while True:
    packet = socket_read_binary_integer(1)
    if packet[0] == 0:
      textmsg("Received nothing")
    elif packet[0] > 1:
      textmsg("Received too many things")
    else:
      # PRINT OUT THE RECIEVED PACKET
      textmsg(packet)
      ###
      mtype = packet[1]
      if mtype == MSG_TEST:
        textmsg("Received Test Message")
        break
      end
    end
    sleep(.008)
  end
end
testProg()'''

    prog.strip()
    print "Sending program: " + prog
    if type(prog) != bytes:
        prog = prog.encode()
    sock.send(prog+b'\n')

    sleep(2)
    
    MSG_TEST = 3

    while not rospy.is_shutdown():
        sock.send(struct.pack("!i", MSG_TEST))
        # Test to see if socket is open... works
        # sock.send("stopj(.1)\n")
        sleep(.01)

    print 'sent'
    sock.close() 
