#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt 
import control as ctrl 
import sympy as sp
import os
import pickle

class InvertedPendulum:

    def __init__(self):
        self.g = 9.81
        self.m_c = 2.7
        self.m_p = 0.85955                    
        self.l_p = 0.5
        self.J_p = 0.076
        self.friction = True

        if (self.friction == True):
            self.b = 0.1
        else:
            self.b = 0.0

    def diff_model(self):

        # p = x[0]
        # theta = x[1]
        # dp = x[2]
        # dtheta = x[3]

        # dx = np.zeros(4)
        ## without friction
        # dx[0] = dp
        # dx[1] = dtheta
        # dx[2] = (-self.l_p*self.m_p*np.sin(theta)*dtheta**2 + self.g*self.m_p*np.sin(2*theta)/2 + F)/(self.m_c + self.m_p*np.sin(theta)**2)
        # dx[3] = (self.g*(self.m_c + self.m_p)*np.sin(theta) - (self.l_p*self.m_p*np.sin(theta)*dtheta**2 - F)*np.cos(theta))/(self.l_p*(self.m_c + self.m_p*np.sin(theta)**2))
 
        # Define equations
        g, Mc, Mp, b, J, l, F, p, dp, ddp, th, dth, ddth = sp.symbols('g Mc Mp b J l F p dp ddp th dth ddth', real=True)

        fzero = [(Mc + Mp) * ddp + b * dp + Mp * l * ddth * sp.cos(th) - Mp * l * dth**2 * sp.sin(th) - F,
             (J + Mp * (l)**2) * ddth - Mp * g * l  * sp.sin(th) + Mp * l * ddp * sp.cos(th)]

        # Solve for dds and ddth
        sol = sp.solve(fzero, [ddp, ddth])
        ddp = sp.simplify(sol[ddp])
        ddth = sp.simplify(sol[ddth])

        # Define x, u, and xdot
        u = F
        x = sp.Matrix([p, th,dp, dth])
        xdot = sp.Matrix([dp, dth,ddp, ddth])
        # xdot_sp = sp.Matrix([dp,dth , ddp, ddth])

        # Subtitute parameters
        xdot = xdot.subs({J:self.J_p, Mc:self.m_c,Mp:self.m_p,g:self.g,l:self.l_p,b:self.b})

        return xdot
    
    def diff_model_subs(self,diff_eq,state,u):
        F, p, dp, ddp, th, dth, ddth = sp.symbols('F p dp ddp th dth ddth', real=True)

        sub_diff_eq = diff_eq.subs({p:state[0],th:state[1] ,dp:state[2] ,dth:state[3] , F : u})

        return sub_diff_eq
    

    def load_or_generate_differential_eq(self):
        if os.path.exists('nonlinear_differential_equation.pkl'):
            # Load the function from the pickle file
            with open('nonlinear_differential_equation.pkl', 'rb') as f:
                xdot_function = pickle.load(f)
            print("Loaded nonlinear_differential_equation from pickle file.")
        else:
            # Generate xdot and save it to the pickle file
            xdot_function = self.diff_model()
            with open('nonlinear_differential_equation.pkl', 'wb') as f:
                pickle.dump(xdot_function, f)
            print("Generated and saved nonlinear_differential_equation to pickle file.")

        return xdot_function

    def ss_model(self):

        alpha = (self.m_c + self.m_p)*(self.J_p + self.m_p*self.l_p**2)- self.m_p**2*self.l_p**2
        
        A = np.array([ 
                [0, 0, 1, 0],
                [0, 0, 0, 1],
                [0, -(self.g*self.m_p**2*self.l_p**2)/alpha, -((self.J_p+self.m_p*self.l_p**2)*self.b)/alpha, 0],
                [0, (self.m_p*self.g*self.l_p*(self.m_c + self.m_p))/alpha, -(self.g*self.m_p*self.l_p*self.b)/alpha, 0]
            ])

        B = np.array([
                [0],
                [0],
                [(self.J_p + self.m_p*self.l_p**2)/alpha],
                [-(self.m_p*self.l_p)/alpha]
            ])
        
        C = np.array([
                     [1, 0, 0, 0 ],
                     [0, 1, 0, 0]
                     ])
        return A, B, C
    
    def check_controllability(self):
        A,B = self.ss_model()
        print('system rank',np.linalg.matrix_rank(A))
        print('controllability rank',np.linalg.matrix_rank(ctrl.ctrb(A,B)))

