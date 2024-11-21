"""
This file contains multiple different IK solvers, mainly to prove I can write them.

Also allows



"""


import numpy as np
from math import radians, degrees
import matplotlib.pyplot as plt
import time
import random as rand


"""
Gradient descent method to generate joint angles for a given position
Inverse Kinematics  

Adapted from: https://liu.diva-portal.org/smash/get/diva2:1774792/FULLTEXT01.pdf   and https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf

Currently updates the joint angles and moves the robots angles at the same time. - does this matter? I don't intend on doing this then not moving...
"""
def _gradient_descent(robot, goal_pos):


    #Inbuilt to function - will require fiddling
    STEP_SIZE = -1e-12
    #Alpha = learning rate - needs to be tuned
    ALPHA = 3.5
    TOLERANCE = 3
    MAX_ITERATIONS = 5000

    #Get current joint angles
    q = robot.theta_list



    #Generate the current pose based on previously calculated results
    current_pos = robot.get_pos_euler()
    print(f"STARTING POS: {current_pos}")

    e = np.subtract(goal_pos, current_pos)

    print(e)
    iterations = 0

    errors = []
    t = []

    while np.linalg.norm(e) >= TOLERANCE:


        errors.append(np.linalg.norm(e))
        t.append(iterations)

        #Calculate the jacobian
        J = robot.calc_jacobian()
        #print(f"Jacboian: {J}")

        #Transpose the jacobian
        J_T = np.transpose(J)

        gradient = ALPHA * np.matmul(J_T, e)  

        #print(gradient)

        for i in range(len(q)):
            #Modify each joint angle by the calculated step
            step = gradient[i] * STEP_SIZE            

            q[i] = q[i] + step       



        #CHECK JOINT LIMITS HERE
        q = robot.check_joint_limits(q)

        #Update the joint angles of the robot 
        robot.update_joint_angles(q)

        #Get the new pos of the robot
        current_pos = robot.get_pos_euler()


        #Calculate new pose difference
        e = np.subtract(goal_pos, current_pos)

        print(e)

        

        #Increase iteration count
        iterations = iterations + 1

        if iterations >= MAX_ITERATIONS:
            print("MAX ITERATIONS PASSED")
            break

    #If solved print the new joint angles
    print(f"ENDING POS: {current_pos}")
    print(f"DIFF: {e} \n NORM: {np.linalg.norm(e)}")    

    plt.plot(t, errors)

    plt.show()

    return q 
    

"""
Pseudoinverse method - similar to gradient descent but using the pseudoinverse rather than the transpose
"""

def _psuedo_inverse(robot, goal_pos):


    #Inbuilt to function - will require fiddling
    STEP_SIZE = 1e-6
    TOLERANCE = 3
    MAX_ITERATIONS = 5000

    #Get current joint angles
    q = robot.theta_list

    #Generate the current pose based on previously calculated results
    current_pos = robot.get_pos_euler()
    print(f"STARTING POS: {current_pos}")

    e = np.subtract(goal_pos, current_pos)

    print(e)
    iterations = 0

    errors = []
    t = []

    prev_e = np.linalg.norm(e)

    best_e = 100000
 
    while curr_e := (np.linalg.norm(e)) >= TOLERANCE:



        errors.append(np.linalg.norm(e))
        t.append(iterations)

        #Calculate the jacobian
        J = robot.calc_jacobian()
        #print(f"Jacboian: {J}")


        #Calculate the psuedo-inverse jacobian
        J_psuedo_inv = np.linalg.pinv(J)

        #print(J_psuedo_inv)


        for i in range(len(q)):

            delta_q = np.matmul(J_psuedo_inv[i], e)

            #print(delta_q)

            #Modify each joint angle by the calculated step
            #Clip the angles so that we don't move too far
            step = delta_q * STEP_SIZE            

            q[i] = q[i] + step       



        #CHECK JOINT LIMITS HERE
        q = robot.check_joint_limits(q)

        #Update the joint angles of the robot 
        robot.update_joint_angles(q)

        #Get the new pos of the robot
        current_pos = robot.get_pos_euler()
      
        

        #Calculate new pose difference
        e = np.subtract(goal_pos, current_pos)

       

        #Increase iteration count
        iterations = iterations + 1

        if iterations >= MAX_ITERATIONS:
            print("MAX ITERATIONS PASSED")
            break

        
        #If the error hasn't changed 
        if iterations > 1 and prev_e == curr_e:
            #Randomly change one of the joints to see if you can stochastically find a better solution
            q[rand.randint(0, 2)] += radians(rand.randint(-180,180))


        prev_e = curr_e


        #Save the closest position
        if curr_e < best_e:
            best_e = curr_e
            best_angles = q
            best_pos = current_pos

    #If solved print the new joint angles
    print(f"Best POS: {best_pos}")
    print(f"CLOSEST NORM: {best_e}")    

    plt.plot(t, errors)

    plt.show()

    return best_angles
    

