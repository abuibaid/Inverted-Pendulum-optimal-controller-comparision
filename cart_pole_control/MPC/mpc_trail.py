from casadi import * 
import numpy as np
import matplotlib.pyplot as plt 
from model import InvertedPendulum
from colorama import Fore, Back, Style, init
init(autoreset=True)



#########Controller definitions
n_x = 4                          # System order
n_u  = 1                       # num of inputs
Ts = 0.05
N = 10
Q = np.diag([100, 10000, 0, 0]) 
R = 0.001*np.eye(1)
p_max = 1
p_min = -1
th_max = 15.7079632679
th_min = - 15.7079632679

x_min = np.array([[p_min],
                 [th_min],
                 [0],
                 [0],]) 

x_max = np.array([[p_max],
                 [th_max],
                 [0],
                 [0],]) 

u_min = -100
u_max = 100


####### System Model ######
system = InvertedPendulum()
A,B,C = system.ss_model()
## model Discretization
Ad = np.eye(4)+ Ts*A
Bd = Ts*B
###########################




######## Problem formalization###
####### multiple shooting, NLP ##
g=[]
h=[]
X=SX.sym('X',n_x,N+1)
U=SX.sym('U',n_u,N)
# X = []
# U = []
obj = 0
x_desired = np.zeros(4) 
x_0 = np.zeros(4).reshape(-1,1)
x_0[2]=3

states = X[:,0]# SX.sym('x',n,1)
control = U[:,0]#SX.sym('u')
rhs = Ad@states + Bd@control
f=Function('f',[states,control],[rhs])

for j in range(N):
    x = X[:,j]
    x_next=X[:,j+1]#SX.sym('x',4,1)
    u=U[:,j] # SX.sym('u',1,1)
    obj += (x_desired-x).T@Q@(x_desired-x) + u.T@R@u
    g += [x_next-x-f(x,u)]
    # X +=[x_next]
    # U +=[u]


print(Fore.RED +'u: \n',U)
# print(Fore.RED +'length of the vector u: \n', len(U))
print(Fore.RED +'x \n',X)
# print(Fore.RED +'Length of the vector x:\n', len(X))
print(Fore.RED +'Objective function: \n',obj)
print(Fore.RED +'Equality constraints: \n',g)


####### Solver properties #####
# decision_var = [] 
# for i in range(N):
#     for j in range(n_x-1):
#         decision_var[j+i,:] = X[j,i]

opt_var =vertcat(reshape(X,4*(N+1),1), reshape(U,N,1))
# opt_var = np.concatenate((X.reshape(4*(N+1), 1), U.reshape(N, 1)))

nlp_prob = {'f': obj, 'x': opt_var, 'g': vertcat(*g)}

# Number of iterations
iter_end = 500  # 100

options = {}
ipopt_options = {}
ipopt_options["max_iter"] = iter_end
ipopt_options["print_level"] = 0
options["print_time"] = False
# 1e-4        "Acceptable" convergence tolerance (relative).
ipopt_options["acceptable_tol"] = 1e-4
# 1e-6    "Acceptance" stopping criterion based on objective function change.
ipopt_options["acceptable_obj_change_tol"] = 1e-6
ipopt_options["linear_solver"] = 'ma57'  #ma57
options["jit"] = True
ipopt_options['warm_start_init_point'] = 'yes'
ipopt_options['fast_step_computation'] = 'yes'
options["ipopt"] = ipopt_options


solver = nlpsol('solver', 'ipopt', nlp_prob,options)


args = {}
# Define arguments
args = {}
args['p'] = []
args['x0'] = np.zeros(n_x)

#Equality constraints
args['lbg'] = np.zeros((n_x*(N+1),1))
args['ubg'] = np.zeros((n_x*(N+1),1))

# inequality constraints
args['lbg'] = vertcat(args['lbg'], -np.ones(N)*np.inf)
args['ubg'] = vertcat(args['ubg'], np.zeros(N))

# constraints in states
lbx_step = x_min
ubx_step = x_max
lbu_step = u_min
ubu_step = u_max
args['lbx'] = vcat([lbx_step for i in range(0,N+1)] + [lbu_step for i in range(0,N)])
args['ubx'] = vcat([ubx_step for i in range(0,N+1)] + [ubu_step for i in range(0,N)])




        
