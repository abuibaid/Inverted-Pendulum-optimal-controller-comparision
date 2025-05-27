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
# model = InvertedPendulum()


class MPCController():
    def __init__(self,A, B,diff_eq):
        self.Ts = 0.05
        self.Q = np.diag([10000, 100000, 0, 0]) 
        self.R = 0.000001*np.eye(1)
        self.A = A
        self.B = B
        self.diff_eq = diff_eq 

        self.nx = 4                          # System order
        self.nu  = 1                       # num of inputs
        self.N = 50
        self.pos_max = 1
        self.pos_min = -1
        self.th_max = 15.8
        self.th_min = -15.8

        self.x_min = np.array([[self.pos_min],                 # distance in meter
                        [self.th_min],                         # angle in radian
                        [-inf],                                # 
                        [-inf],]) 

        self.x_max = np.array([[self.pos_max],
                        [self.th_max],
                        [inf],
                        [inf],]) 

        self.u_min = -20                          # u is Torque
        self.u_max = 20                           

        self.MPC_Problem()
    def MPC_Problem(self):
        ## model Discretization
        self.Ad = np.eye(4)+ self.Ts*self.A
        self.Bd = self.Ts*self.B
        
       # Define arguments
        self.args = {}
        self.args['p'] = []                                                      # parameters, which include the initial and the reference state of the robot
        self.args['x0'] =[]                                                      # Initial guess of the N multiple shooting 

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
        X=SX.sym('X',self.nx,self.N+1)
        U=SX.sym('U',self.nu,self.N)
        # X = []
        # U = []
        obj = 0
        P = SX.sym('P',self.nx+self.nx,1)#np.zeros(4).reshape(-1,1)

        states = X[:,0]# SX.sym('x',n,1)
        control = U[:,0]#SX.sym('u')
        rhs = self.A@states + self.B@control
        f=Function('f',[states,control],[rhs])

        u_0 = control
        x_desired = P[self.nx:self.nx+self.nx]
        # g = [x_0-f(x_0,u_0)]
        # g = [f(x_0,u_0)-x_0]
        g = [X[:,0]-P[0:self.nx]]
        for j in range(self.N):
            x = X[:,j]
            x_next=X[:,j+1]#SX.sym('x',4,1)
            u=U[:,j] # SX.sym('u',1,1)
            obj += (x_desired-x).T@self.Q@(x_desired-x) + u.T@self.R@u

            k1 = f(x, u)
            k2 = f(x + self.Ts/2*k1, u)
            k3 = f(x + self.Ts/2*k2, u)
            k4 = f(x + self.Ts * k3, u)
            x_next_RK4 = x + (self.Ts / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            g += [x_next-x_next_RK4]

            # g += [x_next-x-f(x,u)]
            
        # print(Fore.RED +'u: \n',U)
        # print(Fore.RED +'x \n',X)
        # print(Fore.RED +'Objective function: \n',obj)
        # print(Fore.RED +'Equality constraints: \n',g)

        opt_var =vertcat(reshape(X,4*(self.N+1),1), reshape(U,self.N,1))
        # opt_var = np.concatenate((X.reshape(4*(N+1), 1), U.reshape(N, 1)))
        # self.p0=np.matrix(([0.5,1,0,0]))
        nlp_prob = {'f': obj, 'x': opt_var, 'g': vertcat(*g),'p':P}

        # Number of iterations
        iter_end = 500  # 100

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

        return u_opt,x_opt


    def commandrate(self):
        time_interval = self.Ts
        publish_rate = int(1/time_interval)
        rate = rospy.Rate(publish_rate)

        return time_interval,rate
    

    # if __name__=='__main__':
    #     from model import InvertedPendulum
    #     MPC_Problem()
        

