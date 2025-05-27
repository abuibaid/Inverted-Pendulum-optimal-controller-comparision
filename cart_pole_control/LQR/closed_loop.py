import rospy
rospy.init_node('close_loop_control')


import sys
from model import InvertedPendulum
from communication import RosCommunication
from controller import LqrController
import numpy as np
from math import pi
from scipy import linalg
import tf.transformations as tr
from colorama import Fore, Back, Style, init



system = InvertedPendulum()
A,B,C = system.ss_model()
# diff_eq = system.diff_model()
diff_eq = system.load_or_generate_differential_eq()
diff_eq = system.diff_model_subs(diff_eq,np.zeros(4),0)

ctrl = LqrController(A,B)
P, K, E = ctrl.lqr()
time_interval, rate = ctrl.commandrate()


ros = RosCommunication()





state = np.zeros(4)
desierd_pole_angular = 0

while not rospy.is_shutdown():
    state = ros.receiveState()
   
    state[1] -= desierd_pole_angular*time_interval
    print(f"pole_angle_sensor : {state[1]}")
    print(f"cart_position_sensor : {state[0]}")
    # u = -np.dot(K, state)
    u = -np.matmul(K, state)
    print(f"cart_actuator : {u}")
    ros.sendingCommand(u)
    # rate.sleep()

