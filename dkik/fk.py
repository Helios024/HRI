import math as mt
import numpy as np
import random
def forward_kinematics():
    ti_j = np.identity(4)
    no_of_joints = 3
    d_list = [0.089159,0,0,0.10915,0.09465,0.0823]
    a_list = [0,-0.425,-0.39225,0,0,0]
    alpha_list = [mt.pi/2,0,0,mt.pi/2,-mt.pi/2,0]
    #theta_list = [random.random(),random.random(),random.random(),random.random(),random.random(),random.random()]
    theta_list = [0.5,0.5,0.5,0.5,0.5,0.5]
    for i in range (no_of_joints):
        t_d = [[mt.cos(theta_list[i]),-1*mt.sin(theta_list[i])*mt.cos(alpha_list[i]),mt.sin(theta_list[i])*mt.sin(alpha_list[i]),a_list[i]*mt.cos(theta_list[i])],
               [mt.sin(theta_list[i]),mt.cos(theta_list[i])*mt.cos(alpha_list[i]),-1*mt.cos(theta_list[i])*mt.sin(alpha_list[i]),a_list[i]*mt.sin(theta_list[i])],
               [0,mt.sin(alpha_list[i]),mt.cos(alpha_list[i]),d_list[i]],
               [0,0,0,1]]
        ti_j = np.dot(ti_j,t_d)
    return ti_j

print(forward_kinematics())
