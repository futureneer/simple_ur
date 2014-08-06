#!/usr/bin/env python
import rospy
import numpy as np
from copy import deepcopy
import urx
import logging
import socket

class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min

        self.set_point=0.0
        self.error=0.0

    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setPoint(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator=0
        self.Derivator=0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator

def check_distance(a,b,val):
    ''' returns true if distance is SMALLER than val'''
    v1 = np.array(a)
    v2 = np.array(b)
    res = np.sum(np.abs(np.subtract(v1,v2)))
    # print res
    if res < val:
        return True
    else:
        return False

if __name__ == "__main__":



    rob = urx.Robot("192.168.1.155", logLevel=logging.INFO)
    print 'connected to robot'
    sock = socket.create_connection(('192.168.1.155', 30003), timeout=0.5)
    print 'created socket'
    if rob:
        realtime_monitor = rob.get_realtime_monitor()
        print 'created realtime monitor connection'
    rospy.sleep(1)

    # for n in range(500):
    #     vels = [0.0, 0.0, 0.05, 0.0, 0.0, 0.0]
    #     acc = .1
    #     vels.append(acc)
    #     timeout = .01
    #     vels.append(timeout)
    #     prog = "speedl([{},{},{},{},{},{}], a={}, t_min={})\n".format(*vels)
    #     # prog = "textmsg({})\n".format(n)
    #     if type(prog) != bytes:
    #         prog = prog.encode()
    #     # print 'sending command ['+str(prog)+']'
    #     sock.send(prog)
    #     print realtime_monitor.get_all_data(wait=False)['tcp'][2]
    #     # sock.recv(1044)
    #     rospy.sleep(.007)

    # rospy.sleep(1)
    # sock.close() 
    # rob.cleanup()


    P = 10.0
    I = 0.0
    D = 0.1

    pids = []
    for i in range(6):
        pids.append(PID(P,I,D))

    ### Get init pose
    # start_pose = current_pose = rob.getl()
    start_pose = current_pose = realtime_monitor.get_all_data(wait=False)['tcp']
    # start_pose = [1.0, 1.0, 1.0, 1.507, 0.0, -1.507]
    # current_pose = [1.0, 1.0, 1.0, 1.507, 0.0, -1.507]

    ### Create Command Pose
    command_pose = list(start_pose)
    command_pose[2] = command_pose[2] + .2

    for p, i in zip(pids, range(6)):
        p.setPoint(command_pose[i])

    print 'Start Pose: '+ str(start_pose)
    print 'End Pose; '+str(command_pose)

    vel_command_last = []
        
    acc = .1
    timeout = .01

    while not check_distance(command_pose,current_pose,.001):
        # Get current values
        current_pose = realtime_monitor.get_all_data(wait=False)['tcp']
        # print current_pose
        vel_command = []
        for p, i in zip(pids, range(6)):
            up = p.update(current_pose[i])
            vel_command.append(up)

        V = []
        for v,i in zip(vel_command,range(6)):
            if v > 0:
                if v > 1.5:
                    vel_command[i] = 1.5
            else:
                if v < -1.5:
                    vel_command[i] = -1.5
        vel_command_last = list(vel_command)
        # print vel_command[2]
        vel_command.append(acc)
        vel_command.append(timeout)
        prog = "speedl([{},{},{},{},{},{}], a={}, t_min={})\n".format(*vel_command)
        if type(prog) != bytes:
            prog = prog.encode()
        # print 'sending command ['+str(prog)+']'
        sock.send(prog)

        # rob.speedl(velocities=vel_command, acc=.5, min_time=0.1, direct=True)
        
        # print vel_command
        # print current_pose
        # result = []
        # for c,v in zip(current_pose,vel_command):
            # result.append(c+v)
        # print result
        # current_pose = list(result)

        rospy.sleep(.005)

    rob.cleanup()



    # Simulate

    # p=PID(.01,0.0,.01)
    # p.setPoint(5.0)
    # start = 0
    # current = start
    # goal = 5
    # while goal-current>.0001:
    #     pid = p.update(current)
    #     current+=pid
    #     print current
    #     print pid
    #     rospy.sleep(.001)
    # print 'done'
    # print goal-current