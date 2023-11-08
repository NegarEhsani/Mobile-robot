"""
This module is created to define the parameters that are used in the main code.
"""
# Third-Party Library Imports
import numpy as np

global torque1, torque2
torque1 = 0               #Input Torque  
torque2 = 0               #Input Torque
m = 10                    #weight of the robot
I = 5                     #interia
R = 0.15                  #half of distance betweens two wheels
r = 0.05                  #Radius of the wheels
d = 0.1                   #the distance of the center of gravity of the robot from the axis of wheels
dt = 0.01
t1 = 6.25
t = np.arange(0, t1+0.01, dt)
Lower_Limit_exy = -5      # the lower limit of xy error membership function
Upper_Limit_exy = 5       # the Upper limit of xy error membership function
Lower_Limit_edxy = -3     # the lower limit of xy second order deprivative of xy error membership function
Upper_Limit_edxy = 3  # the Upper limit of xy second order deprivative of xy error membership function
h_exy = 10 / 6   
h_edxy = 6 / 6
Lower_Limit_e_teta = -1   # the lower limit of teta error membership function
Upper_Limit_e_teta = 1    # the Upper limit of teta error membership function
Lower_Limit_ed_teta = -1  # the lower limit of teta second order deprivative of xy error membership function
Upper_Limit_ed_teta = 1   # the Upper limit of teta second order deprivative of xy error membership function
h_et = 2 / 6
h_edt = 2 / 6
a_pid = 0
b_pid = 1
h_pid = 1
Lower_Limit_alpha = 2
Upeer_Limit_alpha = 5
h_alpha = 1


###########################
  
# The robot should follow circle path with following specification
wr = 1 
# x_r is the path that robot should follow along x axis
x_r = np.empty((1, t.shape[0]))
x_r = 5 + 3 * np.cos(wr * t)
x_r = x_r.reshape(1, -1)

# x_r_d is derivative of x_r which shows velocity along the x axis
x_r_d = np.empty((1, t.shape[0]))
x_r_d = -3 * wr * np.sin(wr * t)
x_r_d = x_r_d.reshape(1, -1)

# x_r_dd is second order derivative of x_r which shows acceleration along the x axis 
x_r_dd = np.empty((1, t.shape[0]))
x_r_dd = -3 * (wr ** 2) * np.cos(wr * t)
x_r_dd = x_r_dd.reshape(1, -1)


# y_r is the path that robot should follow along y axis
y_r = np.empty((1, t.shape[0]))
y_r = 10 + 3 * np.sin(wr * t)
y_r = y_r.reshape(1, -1)

# y_r_d is derivative of y_r which shows velocity along the y axis
y_r_d = np.empty((1, t.shape[0]))
y_r_d = 3 * wr * np.cos(wr * t)
y_r_d = y_r_d.reshape(1, -1)


# y_r_dd is second order derivative of y_r which shows acceleration along the y axis 
y_r_dd = np.empty((1, t.shape[0]))
y_r_dd = -3 * (wr ** 2) * np.sin(wr * t)
y_r_dd = y_r_dd.reshape(1, -1)

# teta is the angle between robot and x axis
teta = np.empty((1, t.shape[0]))
teta = np.pi / 2 + wr * t
teta = teta.reshape(1, -1)

# teta_d shows angular velocity
teta_d = np.empty((1, t.shape[0]))
teta_d = wr * np.ones(teta.shape)
teta_d = teta_d.reshape(1, -1)

teta_dd = np.empty(teta.shape)

kp_min_xy = 1
kp_max_xy = 5.5

kd_min_xy = 0
kd_max_xy = 0.31

kp_min_t = 1
kp_max_t = 5.5

kd_min_t = 0
kd_max_t = 0.3

# Rules are 
rules1 = np.array([[0, 1, 2, 3, 4, 5, 6, 7],
                   [1, 2, 2, 2, 2, 2, 2, 2],
                   [2, 1, 2, 2, 2, 2, 2, 1],
                   [3, 1, 1, 2, 2, 2, 1, 1],
                   [4, 1, 1, 1, 2, 1, 1, 1],
                   [5, 1, 1, 2, 2, 2, 1, 1],
                   [6, 1, 2, 2, 2, 2, 2, 1],
                   [7, 2, 2, 2, 2, 2, 2, 2]])
rules2 = np.array([[0, 1, 2, 3, 4, 5, 6, 7],
                   [1, 1, 1, 1, 1, 1, 1, 1],
                   [2, 2, 2, 1, 1, 1, 2, 2],
                   [3, 2, 2, 2, 1, 2, 2, 2],
                   [4, 2, 2, 2, 2, 2, 2, 2],
                   [5, 2, 2, 2, 1, 2, 2, 2],
                   [6, 2, 2, 1, 1, 1, 2, 2],
                   [7, 1, 1, 1, 1, 1, 1, 1]])
rules3 = np.array([[0, 1, 2, 3, 4, 5, 6, 7],
                   [1, 1, 1, 1, 1, 1, 1, 1],
                   [2, 2, 2, 3, 3, 3, 2, 2],
                   [3, 4, 3, 3, 2, 3, 3, 4],
                   [4, 5, 4, 3, 3, 3, 4, 5],
                   [5, 4, 3, 3, 2, 3, 3, 4],
                   [6, 3, 3, 2, 2, 2, 3, 3],
                   [7, 2, 2, 2, 2, 2, 2, 2]])

# Initial states
v = 0                              # v is linear velocity
w = 0                              # w is angular velocity
k = 0
tetac = np.empty((1,t.shape[0]+1))
tetac[:,0] = np.pi / 2
xc = np.empty((1,t.shape[0]+1))
xc[:,0] = 8.1
yc = np.empty((1,t.shape[0]+1))
yc[:,0] = 10
e1 = 0                    # distance error             
e3 = 0                    # angle error


q = np.empty((3, t.shape[0] + 1))
q[:, 0] = [xc[:,0][0], yc[:,0][0], tetac[:,0][0]]
xc_d = np.empty((1,t.shape[0]))
yc_d = np.empty((1,t.shape[0]))
tetac_d = np.empty((1,t.shape[0]+1))
e = np.empty((t.shape[0],6))

qdd = np.empty((3, t.shape[0]))

kp1 = np.empty((1, t.shape[0]))
kd1 = np.empty((1, t.shape[0]))
ki1 = np.empty((1, t.shape[0]))

kp2 = np.empty((1, t.shape[0]))
kd2 = np.empty((1, t.shape[0]))
ki2 = np.empty((1, t.shape[0]))

U1 = np.empty((1, t.shape[0]))                     # U1 is control signals to control linear velocity (xy)
U2 = np.empty((1, t.shape[0]))                     # U2 is control signals to control angular velocity(teta)

tt1 = np.empty((1, t.shape[0]))
tt2 = np.empty((1, t.shape[0]))   
