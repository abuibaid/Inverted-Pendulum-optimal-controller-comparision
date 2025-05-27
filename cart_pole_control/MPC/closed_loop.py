import rospy
rospy.init_node('close_loop_control')


import sys
from model import InvertedPendulum
from communication import RosCommunication
# from linearcontroller import MPCController
from nonlinearcontroller import MPCController
# from learningcontroller import LearningMPCController
import numpy as np
from math import pi
from scipy import linalg
import tf.transformations as tr
from colorama import Fore, Back, Style, init



system = InvertedPendulum()
A,B,C = system.ss_model()
# diff_eq = system.diff_model()
diff_eq = system.load_or_generate_differential_eq()
# diff_eq_test = system.diff_model_subs(diff_eq,np.zeros(4),0)

ctrl = MPCController(A,B,diff_eq)
# ctrl = LearningMPCController()
desierd_pole_angular = 0
time_interval, rate = ctrl.commandrate()

ros = RosCommunication()
rate = ros.commandrate()



initial_states = np.zeros((4*(50+1),1))
initial_control = np.zeros(1*(50))
state = np.zeros(4)
desired_state = np.zeros(4)

while not rospy.is_shutdown():
    state = ros.receiveState()
   
    # state[1] -= desired_state[1]*time_interval
    print(f"pole_angle_sensor : {state[1]}")
    print(f"cart_position_sensor : {state[0]}")
    # u = -np.dot(K, state)
    opt_control,opt_states = ctrl.MPC_solver(state,desired_state,initial_control,initial_states)
    
    # opt_control = ctrl.learningMPC_solver(state,desired_state,initial_control,initial_states)
    print(f"cart_actuator : {float(opt_control[0,0])}")
    ros.sendingCommand(float(opt_control[0,0]))
    initial_control = opt_control
    # initial_states = opt_states
    rate.sleep()

