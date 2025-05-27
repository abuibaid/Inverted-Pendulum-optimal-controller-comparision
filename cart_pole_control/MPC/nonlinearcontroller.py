import rospy
import sys
from model import InvertedPendulum
# from communication import RosCommunication
import numpy as np
from math import pi
from scipy import linalg
import tf.transformations as tr
from colorama import Fore, Back, Style, init
from casadi import * 
init(autoreset=True)
import pandas as pd

class MPCController():
    def __init__(self,A, B,diff_eq):
        self.Ts = 0.05
        self.Q = np.diag([100000, 10000000, 0, 0]) 
        self.R = 0.0001*np.eye(1)
        self.A = A
        self.B = B
        self.diff_eq = diff_eq 

        self.nx = 4                          # System order
        self.nu  = 1                       # num of inputs
        self.N = 50
        self.pos_max = 1
        self.pos_min = -1
        self.th_max = 15.8
        self.th_min =  -15.8

        self.x_min = np.array([[self.pos_min],                 # distance in meter
                        [self.th_min],                         # angle in radian
                        [-inf],                                # 
                        [-inf],]) 

        self.x_max = np.array([[self.pos_max],
                        [self.th_max],
                        [inf],
                        [inf],]) 

        
        self.u_max = 20  
        self.u_min = -20                          # u is Torque                         

        self.MPC_Problem()
    def MPC_Problem(self):
        ##### Nonlinear
        x1 = SX.sym('x1')
        x2 = SX.sym('x2')
        x3 = SX.sym('x3')
        x4 = SX.sym('x4')
        F = SX.sym('F')
        xdot1 = x3
        xdot2 = x4
        xdot3 =(0.2908875*F - 0.02908875*x3 + 0.1250161753125*x4**2*sin(x2) - 0.905985630815625*sin(2*x2))/(0.184706550625*sin(x2)**2 + 0.85072205)
        xdot4 = 0.429775*(-F*cos(x2) + 0.1*x3*cos(x2) - 0.2148875*x4**2*sin(2*x2) + 34.9191855*sin(x2))/(0.184706550625*sin(x2)**2 + 0.85072205)
        xdot =vertcat(xdot1,xdot2,xdot3,xdot4)
        states =vertcat(x1,x2,x3,x4)
        control = F
        f_non=Function('f_non',[states,control],[xdot])
        ###########################

       # Define arguments
        self.args = {}
        self.args['p'] = []                                                      # parameters, which include the initial and the reference state of the robot
        self.args['x0'] = []                                                     # Initial guess of the N multiple shooting 

        #Equality constraints
        self.args['lbg'] = np.zeros((self.nx*(self.N+1),1))
        self.args['ubg'] = np.zeros((self.nx*(self.N+1),1))

        # inequality constraints
        # self.args['lbg'] = vertcat(self.args['lbg'], -np.ones(self.N)*np.inf)
        # self.args['ubg'] = vertcat(self.args['ubg'], np.zeros(self.N))

        # Boundries constraints in states and controls
        lbx_step = self.x_min
        ubx_step = self.x_max
        lbu_step = self.u_min
        ubu_step = self.u_max
        self.args['lbx'] = vcat([lbx_step for i in range(0,self.N+1)] + [lbu_step for i in range(0,self.N)])
        self.args['ubx'] = vcat([ubx_step for i in range(0,self.N+1)] + [ubu_step for i in range(0,self.N)])


        ######## Problem formalization###
        ####### multiple shooting, NLP ##
        self.X=SX.sym('X',self.nx,self.N+1)
        self.U=SX.sym('U',self.nu,self.N)
        obj = 0
        P = SX.sym('P',self.nx+self.nx,1)#np.zeros(4).reshape(-1,1)

        states = self.X[:,0]# SX.sym('x',n,1)
        control = self.U[:,0]#SX.sym('u')
        # rhs = self.Ad@states + self.Bd@control
        # f=Function('f',[states,control],[rhs])

        x_0=P[0:self.nx]
        u_0 = control
        x_desired = P[self.nx:self.nx+self.nx]
        # g = [x_0-self.Ts*f_non(x_0,u_0)]
        # g = [x_0-f_non(x_0,u_0)]
        g = [states-x_0]
        for j in range(self.N):
            x = self.X[:,j]
            x_next=self.X[:,j+1]#SX.sym('x',4,1)
            u=self.U[:,j] # SX.sym('u',1,1)
            obj += (x_desired-x).T@self.Q@(x_desired-x) + u.T@self.R@u
            # Rung Kutta
            k1 = f_non(x, u)
            k2 = f_non(x + self.Ts/2*k1, u)
            k3 = f_non(x + self.Ts/2*k2, u)
            k4 = f_non(x + self.Ts * k3, u)
            x_next_RK4 = x + (self.Ts / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            ####
            g += [x_next-x_next_RK4]


            # g += [x_next-x-self.Ts*f_non(x,u)]
 

        # print(Fore.RED +'u: \n',U)
        # print(Fore.RED +'x \n',X)
        # print(Fore.RED +'Objective function: \n',obj)
        # print(Fore.RED +'Equality constraints: \n',g)

        opt_var =vertcat(reshape(self.X,4*(self.N+1),1), reshape(self.U,self.N,1))
        # opt_var = np.concatenate((X.reshape(4*(N+1), 1), U.reshape(N, 1)))
        # self.p0=np.matrix(([0.5,1,0,0]))
        nlp_prob = {'f': obj, 'x': opt_var, 'g': vertcat(*g),'p':P}

        # Number of iterations
        iter_end = 2000  # 100

        options = {}
        ipopt_options = {}
        ipopt_options["max_iter"] = iter_end
        ipopt_options["print_level"]= False
        options["print_time"] = False
        # 1e-4        "Acceptable" convergence tolerance (relative).
        ipopt_options["acceptable_tol"] = 1e-4
        # 1e-6    "Acceptance" stopping criterion based on objective function change.
        ipopt_options["acceptable_obj_change_tol"] = 1e-6
        # ipopt_options["linear_solver"] = 'ma57'  #ma57
        options["jit"] = True
        ipopt_options['warm_start_init_point'] = 'yes'
        ipopt_options['fast_step_computation'] = 'yes'
        options["ipopt"] = ipopt_options
        # opts = {
        #             'ipopt': {
        #                 'max_iter': 2000,
        #                 'print_level': 0,
        #                 'acceptable_tol': 1e-8,
        #                 'acceptable_obj_change_tol': 1e-6
        #             },
        #             'print_time': 0
        #         }


        self.solver = nlpsol('solver', 'ipopt', nlp_prob,options)


        # return nlpsol('solver', 'ipopt', nlp_prob,options)

    def MPC_solver(self,states,desired_state,initial_controls,intial_state):

        # Intial guess from state from system
        x0 = reshape(intial_state,(-1,1))
        u0 = reshape(initial_controls,(-1,1))
        self.args['x0'] = vcat([x0,u0 ])

        self.args['p'] =  np.vstack([states.reshape(-1,1),desired_state.reshape(-1,1)])

        # solve nlp
        sol=self.solver(x0 = self.args['x0'], lbx = self.args['lbx'], ubx = self.args['ubx'], lbg = self.args['lbg'], ubg = self.args['ubg'],p= self.args['p'] )
        print(Fore.GREEN +(str(self.solver.stats()['return_status'])))
        # Raise an exception is from is infeasible
        if str(self.solver.stats()['return_status']) != 'Solve_Succeeded':
            print(Fore.RED +'###############################################################')
            print(Fore.RED +'###############################################################')
            print(Fore.RED +'################### Solver did not converge ###################')
            print(Fore.RED +'###############################################################')
            print(Fore.RED +'###############################################################')
            #raise RuntimeError ('Solution:Infeasible, The solver did not converge') 
     
        x_opt = np.array(sol['x'][0:self.nx*(self.N+1)]).reshape((self.N+1,self.nx)).T
        u_opt = np.array(sol['x'][self.nx*(self.N+1):]).reshape((self.N,self.nu)).T
        
        

        # Update 
        # print(Fore.BLUE +'states = \n',  x_opt)
        # print(Fore.BLUE +'controls = \n', u_opt)
        print(Fore.BLUE +'objValue =\n',  sol['f'])


        # sol_traj = pd.DataFrame(np.vstack([x_opt[:,:-1], u_opt]).T, columns=['pos', 'theta', 'dpos', 'dtheta', 'F'])
        # sol_traj[['F', 'pos']].plot()
        # sol_traj[['F', 'theta']].plot()
        # sol_traj[['dpos']].plot()
        # sol_traj[['dtheta']].plot()


        return u_opt,x_opt



    

    # if __name__=='__main__':
    #     from model import InvertedPendulum
    #     MPC_Problem()
        

