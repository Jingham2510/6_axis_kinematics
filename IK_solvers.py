"""
This file contains multiple different IK solvers, mainly to prove I can write them.

Also allows



"""


import numpy as np
from math import radians, degrees
import matplotlib.pyplot as plt


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

        #Update the joint angles of the robot - whilst also moving it 
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

        #Calculate the psuedo-inverse jacobian
        J_psuedo_inv = np.linalg.pinv(J)

        #print(J_psuedo_inv)


        for i in range(len(q)):

            delta_q = np.matmul(J_psuedo_inv[i], e)

            #print(delta_q)

            #Modify each joint angle by the calculated step
            step = delta_q * STEP_SIZE            

            q[i] = q[i] + step       



        #CHECK JOINT LIMITS HERE
        q = robot.check_joint_limits(q)

        #Update the joint angles of the robot - whilst also moving it 
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

    #If solved print the new joint angles
    print(f"ENDING POS: {current_pos}")
    print(f"DIFF: {e} \n NORM: {np.linalg.norm(e)}")    

    plt.plot(t, errors)

    plt.show()

    return q 
    




"""
NOTE: Commented out IK solvers are because they aren't suitable // also don't really know how to fix them

Implementatoin of the FABRIK solution - based on the pseudocode presented in the original papers
developed at cambridge btw

target_pos format = [X,Y,Z] - in reference to the robots origin frame (which is currently the base frame)

CURRENTLY A BIT STUCK - FABRIK finds smooth joint movement, not joint angles! there has to be a modification though?


def FABRIK_IK(self, target_pos, tolerance):

    #Calculate the distance between the base and the target position 
    #Technically because we are doing everything in reference to the base frame so we just need the desired points coordinates
    target_distance = sqrt(pow(target_pos[0] - self._ORIGIN_FRAME[0], 2) 
                            + pow(target_pos[1] - self._ORIGIN_FRAME[1], 2)
                            + pow(target_pos[2] - self._ORIGIN_FRAME[2], 2))

    #Get the current positions of the joints (mathematically)
    curr_joint_pos = self._get_all_joint_pos()


    #Check if the target is reachable
    if target_distance > self.robot_length:
        #Unreachable
        #Iterate through joint angle positions
        for i in range(len(self.theta_list)):
            #Find distance between the target and the current joint position
            r = sqrt(pow(target_pos[0] - curr_joint_pos[i][0], 2) 
                            + pow(target_pos[1] - curr_joint_pos[i][1], 2)
                            + pow(target_pos[2] - curr_joint_pos[i][2], 2))
            
            #calculate the links distance to the next link and divide by the distance to the target
            lmbda = sqrt(pow(self.link_list[i].a + self.link_list[i].d, 2))/r

            #calulcate the new joint angle - I don't think FABRIK is built for this
            #self.theta_list[i] = (1 - lmbda)*self.theta_list[i] + lmbda*target_pos  

    return


Calcuakte joint angles utilising the geometric IK method
We only care about Elbow Up config due to the robot and the experimental setup

USEFUL: https://www.youtube.com/watch?v=D93iQVoSScQ

Again - this might not be the right solution

def geometric_IK(self, target_pos):

    #Get the angle of the base - verified!
    theta_1 = atan2(target_pos[1], target_pos[0])


    phi_2 = atan2((target_pos[2]-self._DH_PARAMS[0][2]),(sqrt(target_pos[0] + target_pos[1])))
    #Its a funny one because the joint lengths are spread differently to make the DH work
    

    #So including an extra length to make sure we reach the end
    phi_1 = acos(1)

    #ptheta_2 = phi_2 - phi_1 ;phi_2 = atan(r2/r1), phi_1 = 
    theta_2 =  phi_2 - phi_1

    return

"""