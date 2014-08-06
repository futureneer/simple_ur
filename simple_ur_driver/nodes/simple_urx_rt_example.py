#!/usr/bin/env python
import rospy
import numpy as np
from copy import deepcopy
import urx
import logging
import socket

if __name__ == "__main__":

    rob = urx.Robot("192.168.1.155", logLevel=logging.INFO)
    print 'connected to robot'
    sock = socket.create_connection(('192.168.1.155', 30003), timeout=0.5)
    print 'created socket'
    if rob:
        realtime_monitor = rob.get_realtime_monitor()
        print 'created realtime monitor connection'
    rospy.sleep(1)

    for n in range(500):
        vels = [0.0, 0.0, 0.05, 0.0, 0.0, 0.0]
        acc = .1
        vels.append(acc)
        timeout = .01
        vels.append(timeout)
        prog = "speedl([{},{},{},{},{},{}], a={}, t_min={})\n".format(*vels)
        # prog = "textmsg({})\n".format(n)
        if type(prog) != bytes:
            prog = prog.encode()
        # print 'sending command ['+str(prog)+']'
        sock.send(prog)
        print realtime_monitor.get_all_data(wait=False)['tcp'][2]
        # sock.recv(1044)
        rospy.sleep(.007)

    rospy.sleep(1)
    sock.close() 
    rob.cleanup()