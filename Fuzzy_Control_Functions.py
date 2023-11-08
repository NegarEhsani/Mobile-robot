
"""
This module is created to keep the functions that are needed in project.

"""

# Standard Library Imports
# None
# Third-Party Library Imports
import numpy as np
import matplotlib.pyplot as plt

# Local or Custom Imports
from ParameterHandler import*


def coefficients(rules, a1, b1, h1, a2, b2, h2, e, edot, a_pid, h_pid):
    """
    coefficients Function is defined to calculate PID coefficients
    based on Singleton Fuzzifier, Center average defuzzifier 
    and mamdani fuzzy inference engine.
    
    """
    s_num1 = 0
    s_num2 = 0
    s_den1 = 0
    s_den2 = 0
    j_end_e = int(np.ceil((b1 - a1) / h1)) + 1
    j_end_ed = int(np.ceil((b2 - a2) / h2)) + 1
    for j1 in range(1, j_end_e + 1):
        mem1 = memf_tri(a1, b1, e, j1, h1)
        for j2 in range(1, j_end_ed + 1):
            mem2 = memf_tri(a2, b2, edot, j2, h2)
            y_bar = Y_bar(rules, j1, j2, a_pid, h_pid)
            s_num2 = s_num2 + mem1 * mem2 * y_bar
            s_den2 = s_den2 + mem1 * mem2
        s_num1 = s_num1 + s_num2
        s_den1 = s_den1 + s_den2
        s_num2 = 0
        s_den2 = 0
    K = s_num1 / s_den1
    return K

def Y_bar(rules, j1, j2, a_pid, h_pid):
    """
    this function checks that the membership function of each error mode and error derivative are in accordance 
    with which rule and the center of the membership function obtains the corresponding coefficient.
    
    """
    
    for i in range(1, rules.shape[0] + 1):
        if i == j1:
            k1 = i
            break
    for i in range(1, rules.shape[1] + 1):
        if i == j2:
            k2 = i
            break
    y_bar = a_pid + h_pid * (rules[k1, k2] - 1)
    return y_bar



def dynamics1(x):
    """
    dynamics function is written to solve dynamic equations of the robot.
    
    """
    global torque1, torque2      # torque1 and torque 2 are input torques of the system
    m = 10                       # weight of the robot
    I = 5                        # I is interia
    R = 0.15                     # R is half of distance betweens two wheels
    r = 0.05                     # r is Radius of the wheels 
    d = 0.1                      # d id the distance of the center of gravity of the robot from the axis of wheels
    
    # M is symmetric and positive matrix of mass and inertia
    M = np.array([[m, 0, m * d * np.sin(x[4])],                     
                  [0, m, -m * d * np.cos(x[4])],
                  [m * d * np.sin(x[4]), -m * d * np.cos(x[4]), I]])
    
    # V is matrix of coriolis and centrifugal forces
    V = np.array([[m * d * (x[5] ** 2) * np.cos(x[4])],
                  [m * d * (x[5] ** 2) * np.sin(x[4])],
                  [0]])
    
    # B is input matrix
    B = (1 / r) * np.array([[np.cos(x[4]), np.cos(x[4])],
                            [np.sin(x[4]), np.sin(x[4])],
                            [R, -R]])
    
    A = np.array([[-m * np.sin(x[4]) * (x[1] * np.cos(x[4]) + x[3] * np.sin(x[4])) * x[5]],
                  [m * np.cos(x[4]) * (x[1] * np.cos(x[4]) + x[3] * np.sin(x[4])) * x[5]],
                  [-d * m * (x[1] * np.cos(x[4]) + x[3] * np.sin(x[4])) * x[5]]])
    
    torque= np.array([torque1, torque2]).reshape(2,1)
    
    # dd is representative of second order derivative
    qdd = np.linalg.inv(M) @ (B @ torque + A - V.reshape(3, 1))
    return qdd


def memf_tri(a, b, x, j, h):
    """
    The following function is to define triangle Membership Function
    
    """
    ej = a + h * (j - 1)
    if x <= a or x >= b:
        mem = 1
    else:
        if x >= ej and x < ej + h:
            mem = -(1 / h) * (x - ej) + 1
        elif x < ej and x > ej - h:
            mem = (1 / h) * (x - ej) + 1
        else:
            mem = 0
    return mem



def plt_mem(a, b, h):
    """
    Plot_mem is a function to plot triangular membership functions.
    
    """
    j_end = int(np.ceil((b - a) / h)) + 1
    k = 1
    n = np.arange(a, b+0.001, 0.001)
    mem = np.empty(n.shape[0])
    e = 0.001
    for j in range(1, j_end +1):
        for x in np.arange(a, b+0.001, e):
            mem[k - 1] = memf_tri(a, b, x, j, h)
            k = k + 1
        x = np.arange(a, b+0.001, e)
        plt.plot(x, mem, 'b')
        plt.axis([a, b, 0, 1])
        k = 1
        
# Plots 
def plot_membership(lower_limit, upper_limit, h, title, x_label, y_label):
    plt_mem(lower_limit, upper_limit, h)
    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.grid(False)
    plt.show()

def plot_simple(t, data, title, x_label, y_label):
    plt.plot(t, data[0])
    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.grid(False)
    plt.show()
