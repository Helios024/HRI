import math as mt
import numpy as np
from sympy import symbols, Eq, solve, sin, cos, sqrt
import random

def forward_kinematics(k,j,theta_list): #kRj
    ti_j = np.identity(4)
    d_list = [0.089159,0,0,0.10915,0.09465,0.0823]
    a_list = [0,-0.425,-0.39225,0,0,0]
    alpha_list = [mt.pi/2,0,0,mt.pi/2,-1*mt.pi/2,0]
    for i in range (k,j):
        t_d = [[mt.cos(theta_list[i]),-1*mt.sin(theta_list[i])*mt.cos(alpha_list[i]),mt.sin(theta_list[i])*mt.sin(alpha_list[i]),a_list[i]*mt.cos(theta_list[i])],
               [mt.sin(theta_list[i]),mt.cos(theta_list[i])*mt.cos(alpha_list[i]),-1*mt.cos(theta_list[i])*mt.sin(alpha_list[i]),a_list[i]*mt.sin(theta_list[i])],
               [0,mt.sin(alpha_list[i]),mt.cos(alpha_list[i]),d_list[i]],
               [0,0,0,1]]
        ti_j = ti_j @ t_d
    return ti_j

def cameratransform(k,j,theta_list):
    getframe = forward_kinematics(k,j,theta_list)
    cameratrans = [[1,0,0,0],[0,1,0,-10],[0,0,1,-18],[0,0,0,1]]
    return getframe @ cameratransform

def inverse_kinematics(x,y,z,r):
    q1_q3 = calculate_transl(x,y,z) #Position
    print("T0_3 after translation:\n", forward_kinematics(0,3,[*q1_q3,0,0,0]))
    q4_q6 = calculate_rot([q1_q3[0],q1_q3[1],q1_q3[2]],r) #Rotate
    print("T0_6 after orientation:\n", forward_kinematics(0,6,[*q1_q3,*q4_q6]))
    return 0
    # extransl = calExcessTrans(x,y,z,[*q1_q3,*q4_q6]) #calculating excess translation 3P6
    # print("Excess Translation:(x'',y'',z'')",extransl)
    # x,y,z = x + extransl[0],y + extransl[1],z + extransl[2] #re-adjust x,y,z
    # print(x,y,z)
    # q1toq3 = calculate_transl(x,y,z) #re-position
    # print("T0_6 after re-translation:\n", forward_kinematics(0,3,[*q1toq3,0,0,0]))
    # q4toq6 = calculate_rot([q1toq3[0],q1toq3[1],q1toq3[2]],r) #re-orientation
    # print("T0_6 after reorientation:\n", forward_kinematics(0,6,[*q1toq3,*q4toq6]))
    # return [*q1toq3,*q4toq6]

def calculate_transl(x,y,z):
    A = 0.4250
    B = 0.3922
    W = z - 0.0892
    q1 = mt.atan2(y,x)
    # Define the variables
    q2 = symbols('q2')
    # Define the equations
    eq1 = Eq(- A*sin(q1)*cos(q2) - sqrt(B**2-A**2*sin(q2)**2 - W**2 - 2*A*sin(q2)*W)*sin(q1),y)
    # Solve the equations
    solutions = solve((eq1), (q2))
    if solutions:
        q2 = solutions[0]
    else:
        print('#'*15)
        eq3 = Eq(- A*sin(q1)*cos(q2) + sqrt(B**2-A**2*sin(q2)**2 - W**2 - 2*A*sin(q2)*W)*sin(q1),y)
        sol = solve((eq3), (q2))
        q2 = sol[0]
    
    q3 = symbols('q3')
    # Define the equations
    eq2 = Eq(W + A*sin(q2) + B*sin(q2+q3),0)
    # Solve the equations
    solutions1 = solve((eq2), (q3))
    q3 = solutions1[0]

    return q1,q2,q3

def calculate_rot(q1_q3,rot_0_6):
    trans_0_3 = forward_kinematics(0,3,q1_q3)   
    rot_0_3 = [[trans_0_3[0,0],trans_0_3[0,1],trans_0_3[0,2]],
               [trans_0_3[1,0],trans_0_3[1,1],trans_0_3[1,2]],
               [trans_0_3[2,0],trans_0_3[2,1],trans_0_3[2,2]]]
    inv_rot_0_3 = np.transpose(rot_0_3)
    rot_3_6 = inv_rot_0_3 @ rot_0_6
    #print(rot_3_6)
    q4_numerical =  mt.atan2(rot_3_6[1,2],rot_3_6[0,2]) + mt.pi
    q5_numerical =  mt.acos(rot_3_6[2,2]) 
    q6_numerical =  mt.atan2(-1*rot_3_6[2,1],rot_3_6[2,0]) 
    
    return q4_numerical,q5_numerical,q6_numerical

def calExcessTrans(x,y,z,q1_q6):
    trans_0_6 = forward_kinematics(0,6,q1_q6)
    extransl = [x-trans_0_6[0,3],y-trans_0_6[1,3],z-trans_0_6[2,3]]
    return extransl

rotmat0_6= [[1,0,0],[0,1,0],[0,0,1]]
print(inverse_kinematics(0.2,0.2,0.2,rotmat0_6))

#The equation for translation only works accurately for smaller numbers of x,y,z
