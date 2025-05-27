import rospy
from hilo_mpc import NMPC, Model
from casadi import *
from bokeh.io import output_notebook, show,export_png
from bokeh.plotting import figure, save
from bokeh.layouts import gridplot
import numpy as np
import warnings
warnings.filterwarnings("ignore")


class LearningMPCController():
    def __init__(self):
        self.N = 28
        self.Ts = 0.05
        self.Q = np.diag([100000, 10000000, 0, 0]) 
        self.R = 0.0001*np.eye(1)
        self.pos_max = 1
        self.pos_min = -1
        self.th_max = 15.8
        self.th_min =  -15.8

        self.learningMPC_Problem()

    def learningMPC_Problem(self):
        # model = Model(plot_backend='bokeh')



        # Constants
        # M = 2.7
        # m = 0.85955
        # l = 0.5
        # h = .5
        # g = 9.81
        # J = 0.076

        # States and algebraic variables
        # x = model.set_dynamical_states(['p', 'th','dp', 'dth'])
        # model.set_measurements(['yp','yth','ydp', 'tdth'])
        # model.set_measurement_equations([x[0], x[1], x[2], x[3]])
        # y = model.set_algebraic_states(['y'])

        # Unwrap states
        # th = x[1]
        # dp = x[2]
        # dth = x[3]

        # Define inputs
        # F = model.set_inputs('F')

        # ODEs
        # xdot = SX.sym('xdot', 4)
        # xdot[0] = dp
        # xdot[1] = dth
        # xdot[2] = (1. / (M + m - m * cos(th)) ) * (m * g * sin(th) - m * l * sin(th) * dth ** 2 + F)
        # xdot[3] = (1. / l) * (xdot[1] * cos(th) + g * sin(th))


        # xdot1 = x3
        # xdot2 = x4
        # xdot3 =(0.2908875*F - 0.02908875*x3 + 0.1250161753125*x4**2*sin(x2) - 0.905985630815625*sin(2*x2))/(0.184706550625*sin(x2)**2 + 0.85072205)
        # xdot4 = 0.429775*(-F*cos(x2) + 0.1*x3*cos(x2) - 0.2148875*x4**2*sin(2*x2) + 34.9191855*sin(x2))/(0.184706550625*sin(x2)**2 + 0.85072205)

        # Algebraic equations (note that it is written in the form rhs = 0)
        # rhs = h + l * ca.cos(theta) - y
        # rhs = th - y

        # Add differential equations
        # model.set_dynamical_equations(xdot)

        # Add algebraic equations
        # model.set_algebraic_equations(rhs)




        model = Model()

        equations = """
        # Constants
        Mc = 2.7
        Mp = 0.85955
        l = 0.5
        g = 9.81
        J = 0.076
        b = 0.1

        # DAE
        # d/dt(p(t)) = dp(t)
        # d/dt(th(t)) = dth(t)
        # d/dt(dp(t)) = (F(k)*J + F(k)*Mp*l**2 + J*Mp*dth(t)**2*l*sin((th(t))) - J*b*dp(t) + Mp**2*(dth(t))**2*l**3*sin((th(t))) - Mp**2*g*l**2*sin(2*(th(t)))/2 - Mp*b*dp(t)*l**2)/(J*Mc + J*Mp + Mc*Mp*l**2 + Mp**2*l**2*sin((th(t)))**2)
        # d/dt(dth(t)) = Mp*l*(-F(k)*cos(th(t)) + Mc*g*sin(th(t)) - Mp*dth(t)**2*l*sin(2*th(t))/2 + Mp*g*sin(th(t)) + b*dp(t)*cos(th(t)))/(J*Mc + J*Mp + Mc*Mp*l**2 + Mp**2*l**2*sin(th(t))**2)
        # 0 = th(t) - y(t)

        d/dt(p(t)) = dp(t)
        d/dt(th(t)) = dth(t)
        d/dt(dp(t)) = (F(k)*J + F(k)*Mp*l**2 + dth(t)**2*l*sin((th(t))) - dp(t) + (dth(t))**2*l**3*sin((th(t))) - Mp**2*g*l**2*sin(2*(th(t)))/2 - dp(t)*l**2)/( J*Mp + l**2 + 2*sin((th(t)))**2)
        d/dt(dth(t)) = (-F(k)*cos(th(t)) + Mc*g*sin(th(t)) - dth(t)**2*l*sin(2*th(t))/2 + Mp*g*sin(th(t)) + dp(t)*cos(th(t)))/( J*Mp + Mc*Mp*l**2 + Mp**2*l**2*sin(th(t))**2)
        0 = th(t) - y(t)
        """
        model.set_equations(equations)


        # Initial conditions
        x0 = [0, 0, 0, 0]

        # Initial guess algebraic states
        # z0 = h + l * ca.cos(x0[1]) - h
        z0 = x0[1]
        #Initial guess input
        u0 = 0.0

        # Setup the model
        dt = self.Ts
        model.setup(dt=dt)


        self.nmpc = NMPC(model)

        self.nmpc.quad_stage_cost.add_states(names=['p', 'th'], ref=[0, 0], weights=[self.Q[0][0], self.Q[1][1]])

        self.nmpc.quad_stage_cost.add_inputs(names='F', weights=self.R)

        self.nmpc.horizon = self.N

        self.nmpc.set_box_constraints(x_ub=[self.pos_max, self.th_max, inf, inf], x_lb=[self.pos_min, self.th_min, -inf, -inf])

        self.nmpc.set_initial_guess(x_guess=x0, u_guess=u0)

        self.nmpc.setup(options={'print_level': 0})

    def learningMPC_solver(self,states,desired_state,initial_controls,intial_state):
        sol = self.nmpc.optimize(states)
        return sol


    def commandrate(self):
        time_interval = self.Ts
        publish_rate = int(1/time_interval)
        rate = rospy.Rate(publish_rate)

        return time_interval,rate


# n_steps = 100

# model.set_initial_conditions(x0=x0, z0=z0)

# self.sol = model.solution

# for step in range(n_steps):
#     u = nmpc.optimize(x0)
#     model.simulate(u=u, steps=1)
#     x0 = sol['x:f']




# output_notebook()
# p_tot = []
# for state in model.dynamical_state_names:
#     p = figure(background_fill_color="#fafafa", width=300, height=300)
#     p.line(x=np.array(sol['t']).squeeze(), y=np.array(sol[state]).squeeze(),
#            legend_label=state, line_width=2)
#     for i in range(len(nmpc.quad_stage_cost._references_list)):
#         if state in nmpc.quad_stage_cost._references_list[i]['names']:
#             position = nmpc.quad_stage_cost._references_list[i]['names'].index(state)
#             value = nmpc.quad_stage_cost._references_list[i]['ref'][position]
#             p.line([np.array(sol['t'][1]).squeeze(), np.array(sol['t'][-1]).squeeze()],
#                    [value, value], legend_label=state + '_ref',
#                    line_dash='dashed', line_color="red", line_width=2)

#     p.yaxis.axis_label = state
#     p.xaxis.axis_label = 'time'
#     p.yaxis.axis_label_text_font_size = "12pt"
#     p.yaxis.major_label_text_font_size = "12pt"
#     p.xaxis.major_label_text_font_size = "12pt"
#     p.xaxis.axis_label_text_font_size = "12pt"

#     p_tot.append(p)


# export_png(gridplot(p_tot, ncols=2), filename="plot.png")
# show(gridplot(p_tot, ncols=2))

