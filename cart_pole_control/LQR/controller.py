import rospy
import sys
from model import InvertedPendulum
from communication import RosCommunication
import numpy as np
from math import pi
from scipy import linalg
import tf.transformations as tr
from colorama import Fore, Back, Style, init
import control as ctrl





class LqrController():
    def __init__(self,A, B):
        self.Ts = 0.05
        self.Q = np.diag([100, 10000, 0, 0]) 
        self.R = 0.001*np.eye(1)
        self.A = A
        self.B = B

    def lqr(self):

        # P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        # K = linalg.inv(self.R).dot(self.B.T).dot(P)
        # E = linalg.eigvals(self.A - self.B.dot(K))


        K,P,E = ctrl.lqr(self.A, self.B, self.Q, self.R)
        return P, K, E


    def commandrate(self):
        time_interval = self.Ts
        publish_rate = int(1/time_interval)
        rate = rospy.Rate(publish_rate)

        return time_interval,rate
        

