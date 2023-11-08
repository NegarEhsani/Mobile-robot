"""
Fuzzy Control of Wheeled mobile robot.
the goal of this project is to control a mobile robot to follow a circular path using fuzzy PID controller.
to control the robot at first step kinematic and dynamic equations of robot are solved and by using them the
next step of the robot is measured. In the second step we define rules based on error and error deprivative 
and by using singleton fuzzifier, Center average deffuzifier and mamdani fuzzy inference we design a PID controller.

"""
# Standard Library Imports
from dataclasses import dataclass

# Third-Party Library Imports
import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt

# Local or Custom Imports
from ParameterHandler import*
from Fuzzy_Control_Functions import*   

# Optional Imports
# from IPython.core.interactiveshell import InteractiveShell
# InteractiveShell.ast_node_interactivity = "all"



# states - kinematic mode
for i in range(0, t.shape[0]):    
    qdot = np.array([[np.cos(tetac[:,k][0]), -d * np.sin(tetac[:,k][0])], [np.sin(tetac[:,k][0]), -d * np.cos(tetac[:,k][0])], [0, 1]]) @ np.array([v, w]).reshape(2,1)
    xc_d[:,k] = qdot[0,0]
    yc_d[:,k] = qdot[1,0]
    tetac_d[:,k] = qdot[2,0]
    
    # xc and yc are Coordinates of the center of mass
    x = np.array([xc[:,k][0], xc_d[:,k][0], yc[:,k][0], yc_d[:,k][0], tetac[:,k][0], tetac_d[:,k][0]])
    
    # error
    e[k,:] = - (x - np.array([x_r[:,k][0], x_r_d[:,k][0], y_r[:,k][0], y_r_d[:,k][0], teta[:,k][0], teta_d[:,k][0]])) 
   
    # dynamic model
    qdd[:,k] = np.ravel(dynamics1([x[0], x[1], x[2], x[3], x[4], x[5]]))
    
    # qdd = qdd.reshape(-1) 
    q[:, k+1] = q[:, k] + qdd[:,k] * (dt ** 2) + np.ravel(np.array([x[1], x[3], x[5]])) * dt

    # error integration (0.999 is forgetting factor)
    e1 = e1 * 0.999 + np.sqrt(e[k][0] ** 2 + e[k][2] ** 2) * dt
    e3 = e3 * 0.999 + e[k][4] * dt

    # coefficients calculations (in this section based of the mentioned fuzzifier, defuzzifier and engines coefficients are measured)
    kpp1 = coefficients(rules1, Lower_Limit_exy, Upper_Limit_exy, h_exy, Lower_Limit_edxy, Upper_Limit_edxy, h_edxy, np.sqrt(e[k][0] ** 2 + e[k][2] ** 2),
                (1 / np.sqrt(e[k][0] ** 2 + e[k][2] ** 2)) * (e[k][1] * e[k][0] + e[k][3] * e[k][2]), a_pid, h_pid)
    
    alpha1 = coefficients(rules2, Lower_Limit_exy, Upper_Limit_exy, h_exy, Lower_Limit_edxy, Upper_Limit_edxy, h_edxy, np.sqrt(e[k][0] ** 2 + e[k][2] ** 2),
                  (1 / np.sqrt(e[k][0] ** 2 + e[k][2] ** 2)) * (e[k][1] * e[k][0] + e[k][3] * e[k][2]), Lower_Limit_alpha, h_alpha)
    
    kdp1 = coefficients(rules3, Lower_Limit_exy, Upper_Limit_exy, h_exy, Lower_Limit_edxy, Upper_Limit_edxy, h_edxy, np.sqrt(e[k][0] ** 2 + e[k][2] ** 2),
                (1 / np.sqrt(e[k][0] ** 2 + e[k][2] ** 2)) * (e[k][1] * e[k][0] + e[k][3] * e[k][2]), a_pid, h_pid)
    
    kpp2 = coefficients(rules1, Lower_Limit_e_teta, Upper_Limit_e_teta, h_et, Lower_Limit_ed_teta, Upper_Limit_ed_teta, h_edt, e[k][4], e[k][5], a_pid, h_pid)
    
    alpha2 = coefficients(rules2, Lower_Limit_e_teta, Upper_Limit_e_teta, h_et, Lower_Limit_ed_teta, Upper_Limit_ed_teta, h_edt, e[k][4], e[k][5], Lower_Limit_alpha, h_alpha)
    
    kdp2 = coefficients(rules3, Lower_Limit_e_teta, Upper_Limit_e_teta, h_et, Lower_Limit_ed_teta, Upper_Limit_ed_teta, h_edt, e[k][4], e[k][5], a_pid, h_pid)
    
    
    kp1[:,k] = (kp_max_xy - kp_min_xy) * kpp1 + kp_min_xy
    kd1[:,k] = (kd_max_xy - kd_min_xy) * kdp1 + kd_min_xy
    ki1[:,k] = ((kp1[:,k] ** 2) / (alpha1 * kd1[:,k]))
    
    kp2[:,k] = (kp_max_t - kp_min_t) * kpp2 + kp_min_t
    kd2[:,k] = (kd_max_t - kd_min_t) * kdp2 + kd_min_t
    ki2[:,k] = ((kp2[:,k] ** 2) / (alpha2 * kd2[:,k]))

    # fuzzy pid control signals
    U1[:,k] = kp1[:,k] * np.sqrt(e[k][0] ** 2 + e[k][2] ** 2) + 0 * e1 + kd1[:,k] * (1 / np.sqrt(e[k][0] ** 2 + e[k][2] ** 2)) * (e[k][1] * e[k][0] + e[k][3] * e[k][2])
    U2[:,k] = kp2[:,k] * e[k][4] + 0 * e3 + kd2[:,k] * e[k][5]

   

    # Torque
    torque1 = (U1[:,k] + U2[:,k]) / 2
    torque2 = (U1[:,k] - U2[:,k]) / 2
    tt1[:,k] = torque1
    tt2[:,k] = torque2

    # v is linear velocity and w is angular velocity
    v = U1[:,k]
    w = U2[:,k]

    # new states ( based on derivative formula next state is calculated )
    xc[:,k+1] = xc[:,k] + xc_d[:,k] * dt
    yc[:,k+1] = yc[:,k] + yc_d[:,k] * dt
    tetac[:,k+1] = tetac[:,k] + tetac_d[:,k] * dt
    k += 1

# Plots
plot_membership(Lower_Limit_exy, Upper_Limit_exy, h_exy, 'membership function error-xy', 'e-xy', 'mu')
plot_membership(Lower_Limit_edxy, Upper_Limit_edxy, h_edxy, 'membership function derivative error-xy', 'edot-xy', 'mu')
plot_membership(Lower_Limit_e_teta, Upper_Limit_e_teta, h_et, 'membership function error-teta', 'e-teta', 'mu')
plot_membership(Lower_Limit_ed_teta, Upper_Limit_ed_teta, h_edt, 'membership function derivative error-teta', 'edot-teta', 'mu')
plot_membership(Lower_Limit_alpha, Upeer_Limit_alpha, h_alpha, 'membership function alpha', 'alpha', 'mu')

plot_simple(t, tt1, 'torque1', 't', 'torque1')
plot_simple(t, tt2, 'torque2', 't', 'torque2')
plot_simple(t, kp1, 'kp1', 't', 'kp1')
plot_simple(t, kd1, 'kd1', 't', 'kd1')
plot_simple(t, kp2, 'kp2', 't', 'kp2')
plot_simple(t, kd2, 'kd2', 't', 'kd2')
plot_simple(t, U1, 'Control Signal - U1(v)', 't', 'U1')
plot_simple(t, U2, 'Control Signal - U2(w)', 't', 'U2')

plt.plot(x_r[0], y_r[0], 'r')
plt.plot(xc[0], yc[0], 'b')
plt.title('x-y')
plt.xlabel('x')
plt.ylabel('y')
plt.grid(False)
plt.show()
