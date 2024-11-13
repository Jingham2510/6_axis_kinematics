   
"""
Gradient descent method to generate joint angles for a given position
Inverse Kinematics  

Adapted from: https://liu.diva-portal.org/smash/get/diva2:1774792/FULLTEXT01.pdf   

Currently updates the joint angles and moves the robots angles at the same time. - should probably change to be analytical
"""
def _gradient_descent(self, goal_pos):

    q = self.theta_list

    #Inbuilt to function - will require fiddling
    STEP_SIZE = 3
    ALPHA = 3.5
    TOLERANCE = 100

    #Generate the current pose based on previously calculated results
    current_pos = [self.pos["X"], self.pos["Y"], self.pos["Z"], self.euler_orient[0][0], self.euler_orient[0][1], self.euler_orient[0][2]]

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


