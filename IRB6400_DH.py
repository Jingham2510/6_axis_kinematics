"""
The DH representaiton of the 6-axis robot

The DH representation (or at least the lengths/angles) should be the same every time, except for the thetas, 
which are a representatoin of the current rotation?

"""

import DH_link
import numpy as np
from math import pi
from math import degrees, radians, sin, asin, cos, atan2, sqrt, acos, copysign
import IK_solvers as IK

class IRB4400_DH:

    """
    CONSTANT - IRB4400 DH parameters - DO NOT MODIFY UNLESS SURE 
    -Dynamic_Analysis_and_Visualization_of_Spatial_Mani suggests modifications to theta values
    These determine all the maths for the forward/inverse kinematics - if incorrect there will be unpredictable behaviour   
    Format is [link_length, link twist, link offset] 
     or       [a,             alpha,        d]
     These are constant for each joint - hence why theta is in a different list


    """

    """     
    #FROM THE IRB ARCHIVE PIM.XML
    _DH_PARAMS = [ [0.24, -pi/2, 0], [1.05, 0, 0], [0.225, -pi/2, 1.52], [0, pi/2, 0], [0, pi/2, 0], [0, 0, 0]]
    _THETA_OFFSETS = [ -pi/2, 0, 0, -pi, 0, 0]
    """
  
    """

    #manual/reading other sources for inspo 
    #https://www.researchgate.net/publication/344042740_Tribo-dynamic_analysis_and_motion_control_of_a_rotating_manipulator_based_on_the_load_and_temperature_dependent_friction_model
    #https://automaticaddison.com/how-to-find-denavit-hartenberg-parameter-tables/
    """

    #A1 SHOULD BE CORRECT?
    #_DH_PARAMS = [[240, pi/2, 800], [1050, 0, 0], [225, -pi/2, 0], [0, -pi/2, 1520], [0, pi/2, 0], [0,0,0]]
    #_THETA_OFFSETS = [0, -pi/2, 0, 0, 0, 0]


    """
    VERIFIED WITH https://tools.glowbuzzer.com/kinviz    
    BELOW ARE THE CORRECT PARAMETERS
    REMEMBER THAT A6 WILL NEED UPDATING DEPENDENT ON THE END AFFECTOR POSITION/ORIENTATION
    
    """

    _DH_PARAMS = [[240, pi/2, 800], [1050, 0, 0], [225, pi/2, 0], [0, -pi/2, 1520], [0,pi/2,0], [0,0,200]]

    _INVERTED = [0, 1, 1, 0, 1, 0]

    _INVERTED = [0, 0, 0, 0, 0, 0]




    #Origin frame in reference to the base of the robot (currently just the base frame)
    _ORIGIN_FRAME = [0, 0, 0]


    #Should be the same everytime
    def __init__(self, theta_list):

        if len(theta_list) != 6:
            print("WARNING: THETA LIST INCORRECT LENGTH")


        self.theta_list = []

        #Check that if the angle is inverted in ABBs control software
        #Some joints don't ofllow classic robots rotation convention (i.e. positive = counter-clockwise)
        for i in range(len(theta_list)):
            if self._INVERTED[i]:
                self.theta_list.append(-theta_list[i])
            else:
                self.theta_list.append(theta_list[i])


        #Calculate fully extended length (not including angle limits - can ignore as we won't reach them in our workspace)
        self.robot_max_length = sqrt(sum(pow((i[0] + i[2]),2) for i in self._DH_PARAMS))




        self.link_list = []


        self._THETA_OFFSETS = [0, pi/2, -self.theta_list[1], 0, 0, 0]

        #Create all of the links with the relevant information
        for i in range(len(theta_list)):
            self.link_list.append(DH_link.DH_link(self._DH_PARAMS[i][0], #link_length
                                                  self._DH_PARAMS[i][1], #link_twist
                                                  self._DH_PARAMS[i][2], #link_offset
                                                  self.theta_list[i] + self._THETA_OFFSETS[i])
                                                  )



        #Calc the current transformation matrix
        self.update_pos_orient()



    """
    Multiply the homogeneous matrices together to determine the transofmration matrix
    """
    def _calc_transform(self):

        #Multiply all of the homogeneus matrices together to get the transformation matrix of oritentation.translation frame 6 
        #in respect to frame 0 (the reference frame)


        self.T1_0 = np.matmul(self.link_list[0].get_hg_mat(), self.link_list[1].get_hg_mat())
        self.T2_0 = np.matmul(self.T1_0, self.link_list[2].get_hg_mat())
        self.T3_0 = np.matmul(self.T2_0, self.link_list[3].get_hg_mat())
        self.T4_0 = np.matmul(self.T3_0, self.link_list[4].get_hg_mat())
        self.T = np.matmul(self.T4_0, self.link_list[5].get_hg_mat())

        print(self.T)



       

        

    """
    Calculate the transform and extract the position and the orientation data
    """
    def update_pos_orient(self):
        #Calculate the trasnform matrix
        self._calc_transform()

        #Extract the position and the orientation from the transform matrix - according to M.Spong
        self.pos = {

            "X": round(self.T[0][3],3), 
            "Y": round(self.T[1][3],3), 
            "Z": round(self.T[2][3],3)
        }


        self.pure_orient_mat = [[self.T[0][0], self.T[0][1], self.T[0][2]], 
                                [self.T[1][0], self.T[1][1], self.T[1][2]], 
                                [self.T[2][0], self.T[2][1], self.T[2][2]]]
        


    """
    Function to calculate every single joints position - useful for IK
    """
    def _get_all_joint_pos(self):

        
        joint_pos = []

        #Do the first calculation outside the loop to avoid a matrix multiplication
        curr_T = self.link_list[0].get_hg_mat()

        joint_pos.append([curr_T[0][3], curr_T[1][3], curr_T[2][3]])
        

        #Go through every joint
        for i in range(1, len(self.link_list)):
            
            curr_T = np.matmul(curr_T, self.link_list[i].get_hg_mat())

            joint_pos.append([curr_T[0][3], curr_T[1][3], curr_T[2][3]])




        return joint_pos
    """
    Updates the joint angles
    """
    def update_joint_angles(self, theta_list):
        
        if len(theta_list) != 6:
            print("WARNING: THETA LIST INCORRECT LENGTH")

        self.theta_list = []

        #Check that the angle isn't inverted in ABBs control software
        for i in range(len(theta_list)):
            if self._INVERTED[i]:
                self.theta_list.append(-theta_list[i])
            else:
                self.theta_list.append(theta_list[i])


        self._THETA_OFFSETS = [0, pi/2, -self.theta_list[1], 0, 0, 0]

        for i in range(len(theta_list)):
            self.link_list[i].update_hg_mat(joint_angle = self.theta_list[i] + self._THETA_OFFSETS[i])

        self.update_pos_orient()



    """
    Getter function for the position of the end affector relative to the base frame
    """
    def get_pos(self):
        return self.pos
    
    """
    Getter function for the orientation of the end affector relative to the base frame
    """
    def get_euler_orient(self):


        
        #Orientation in temrs of euler angles
        #Format is psi (x-axis), theta (y-axis), phi (z-axis)
        #There are two possible solutions 
        #Based on pseudocode from: www.eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf

        
        if abs(self.pure_orient_mat[2][0]) != 1:

    
            #Theres two possible pitches, we just take the first for now 
            pitch = atan2(-self.pure_orient_mat[2][0], sqrt(pow(self.pure_orient_mat[2][1], 2) + pow(self.pure_orient_mat[2][2], 2)))

            roll = atan2(self.pure_orient_mat[2][1]/cos(pitch), self.pure_orient_mat[2][2]/cos(pitch))

            yaw = atan2(self.pure_orient_mat[1][0]/cos(pitch), self.pure_orient_mat[0][0]/cos(pitch))

        else:

            #There are infinite different solutions at this point!
            #We constrain phi to be 0 as we only want one solution
            
            """
            NOTE: In the future it might be important to read the angle/joint data from the robot as opposed to calculating it ourselves because
                 we might end up constraining phi to be something that it isnt! (although technically we will have the same outcome)
            """
            yaw = 0

            if(self.pure_orient_mat[2][0] == -1):
                pitch = pi/2
                roll = yaw + atan2(self.pure_orient_mat[0][1], self.pure_orient_mat[0][2])

            else:
                pitch = -pi/2
                roll = -yaw + atan2(-self.pure_orient_mat[0][1], -self.pure_orient_mat[0][2])


        #All inverted to match robot studio
        self.euler_orient = [roll, yaw, pitch]


        return self.euler_orient


    """
    An alternative to the Euler angles
    Gets the orientation of the TCP in terms of quarternions
    Calculations taken from: https://mecademic.com/insights/academic-tutorials/quaternions-in-industrial-robotics/
    """
    def get_quartenions(self):

        #Only access the reused matrix positions once
        r1_1 = self.pure_orient_mat[0][0]
        r2_2 = self.pure_orient_mat[1][1]
        r3_3 = self.pure_orient_mat[2][2]

        
        q1 = (1/2) * sqrt(1 + r1_1 + r2_2 + r3_3)
        
        #NOTE: Python has no inbuilt sign function, but copysign basically does it for you with a magnitude of 1

        q2 = (1/2) * copysign(1, (self.pure_orient_mat[2][1] - self.pure_orient_mat[1][2])) * sqrt(1 + r1_1 - r2_2 - r3_3)

        q3 = (1/2) * copysign(1, (self.pure_orient_mat[0][2] - self.pure_orient_mat[2][0])) * sqrt(1 - r1_1 + r2_2 - r3_3)

        q4 = (1/2) * copysign(1, (self.pure_orient_mat[1][0] - self.pure_orient_mat[0][1])) * sqrt(1 - r1_1 - r2_2 + r3_3)
        
        quarternions = [q1, q2, q3, q4]

        return quarternions
    
  

    """
    Calculate the current Jacobian based on the current state of the robot

    For a 6-axis robot the Jacobian is a 6x6 matrix.    
    
    """
    def calc_jacobian(self):
        

        J = []
        #Generate the first homogeneous matrix
        trans_matrices = self.link_list[0].get_hg_mat()

        #Get the orientation of the final reference frame
        o_n = [[self.T[0][3]],[self.T[1][3]],[self.T[2][3]]]


        #Calculate each jacobian component for each link
        for i in range(len(self.link_list)):

            print(trans_matrices)

            """
            
            J_i = [z_i-1 x (o_n - o_i-1)]
                  [z_i-1]

            z_i and o_i have already been calculated from the homogenous transformation matrices
            
            """      

            #First 3 elements of third column T^0_i
            z_i_minus_one = np.vstack([[trans_matrices[0][2]], [trans_matrices[1][2]], [trans_matrices[2][2]]])

            #First 3 elements of fourth coloum T^0_i
            o_i_minus_one = np.vstack([[trans_matrices[0][3]], [trans_matrices[1][3]], [trans_matrices[2][3]]])     


            j_top  = np.cross(z_i_minus_one, np.subtract(o_n, o_i_minus_one), axis=0)

            j_bot = z_i_minus_one

            j_i = np.vstack([j_top, j_bot])

            """
            print(f"J_{i} --------------------------")

            print(j_i)           
            """

            J.append(j_i)

            #Calculate the next transofrmation matrix
            trans_matrices = np.matmul(trans_matrices, self.link_list[i].get_hg_mat())


        #Combine the matrices to create a 6x6 jacobian matrix
        jacobian = np.concatenate([i for i in J], axis=1)

        #print(jacobian)

        return jacobian
    

    """
    Calculates new joint angles for a given target position -
    Basically a wrapper that enables the usage of multiple different implementations for finding new angles 

    new_pos format = [X, Y, Z, RX, RY, RZ]

    """
    def calc_new_angles(self, method, goal_pos):

        match method:

            case "gradient descent":
                new_angles = IK._gradient_descent(goal_pos)


            case _:
                print("INVALID METHOD")       
        
        return new_angles






if __name__ == "__main__":


    theta_1 = radians(0) 
    theta_2 = radians(0) 
    theta_3 = radians(0) 
    theta_4 = radians(0) 
    theta_5 = radians(30) 
    theta_6 = radians(0) 

    robot = IRB4400_DH([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6])

    pos = robot.get_pos()

    X = pos["X"]
    Y = pos["Y"]
    Z = pos["Z"]


    print(f"X: {X}  Y: {Y}  Z: {Z}")

 

    print("-----ORIENTATION-----")
    print(f"Euler: {robot.get_euler_orient()}")
    print(f"Quartenion: {robot.get_quartenions()}")

    #print(robot._get_all_joint_pos())


    #robot.FABRIK_IK([1933.205, 100, 1975], 0.5)


    #robot.geometric_IK([X, Y ,Z])

 
    print("---------JACOBIAN----------")

    robot.calc_jacobian()




    #robot.calc_new_angles("gradient descent", [X - 100, Y, Z , 3.1239648281446506, 1.0471975511965979, 3.141592653589793])

