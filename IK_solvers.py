   
import numpy as np

"""
Gradient descent method to generate joint angles for a given position
Inverse Kinematics  

Adapted from: https://liu.diva-portal.org/smash/get/diva2:1774792/FULLTEXT01.pdf   

Currently updates the joint angles and moves the robots angles at the same time. - NEED to change to be analytical
"""
def _gradient_descent(self, curr_pos, goal_pos):

    q = self.theta_list

    #Inbuilt to function - will require fiddling
    STEP_SIZE = 3
    ALPHA = 3.5
    TOLERANCE = 100

    #Generate the current pose based on previously calculated results
    current_pos = [curr_pos[0], curr_pos[1], curr_pos[2], curr_pos[3], curr_pos[4], curr_pos[5]]

    e = np.subtract(goal_pos, current_pos)

    while np.linalg.norm(e) >= TOLERANCE:

        J = self.calc_jacobian()

        J_T = np.transpose(J)

        gradient = ALPHA * J_T * e

        print(gradient)

        for i in range(len(q)):
            q[i] = q[i] + (gradient * STEP_SIZE)

        #CHECK JOINT LIMITS HERE

        #Update the joint angles of the robot - whilst also moving it 
        self.update_joint_angles(q)

        #Get the new pos of the robot
        current_pos = [self.pos["X"], self.pos["Y"], self.pos["Z"], self.euler_orient[0][0], self.euler_orient[0][1], self.euler_orient[0][2]]

        e = np.subtract(goal_pos, current_pos)




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