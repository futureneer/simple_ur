#!/usr/bin/env python
import rospy
import numpy as np
from copy import deepcopy
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
    if res < val:
        return True
    else:
        return False

if __name__ == "__main__":


    # rob = urx.Robot("192.168.1.155", logLevel=logging.INFO)
    # rob.set_tcp((0,0,0,0,0,0))
    # rob.set_payload(0.5, (0,0,0))

    P = 0.01
    I = 0.0
    D = 0.01

    pids = []
    for i in range(6):
        pids.append(PID(P,I,D))

    ### Get init pose
    # start_pose = rob.getl()
    start_pose = [1.0, 1.0, 1.0, 1.507, 0.0, -1.507]
    current_pose = [1.0, 1.0, 1.0, 1.507, 0.0, -1.507]

    ### Create Command Pose
    command_pose = list(start_pose)
    command_pose[2] = command_pose[2] + 1

    for p, i in zip(pids, range(6)):
        p.setPoint(command_pose[i])

    print 'Start Pose: '+ str(start_pose)
    print 'End Pose; '+str(command_pose)

    vel_command_last = []

    # while not check_distance(start_pose,command_pose,.001):
    while True:
        # Get current values
        # current_pose = rob.getl()

        vel_command = []
        for p, i in zip(pids, range(6)):
            up = p.update(current_pose[i])
            vel_command.append(up)

        vel_command_last = list(vel_command)

        # rob.speedl(velocities=vel_command, acc=.1, min_time=.01)
        
        print vel_command
        print current_pose
        result = []
        for c,v in zip(current_pose,vel_command):
            result.append(c+v)
        print result
        current_pose = list(result)

        rospy.sleep(.01)
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