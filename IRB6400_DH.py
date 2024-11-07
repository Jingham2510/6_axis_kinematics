"""
The DH representaiton of the 6-axis robot

The DH representation (or at least the lengths/angles) should be the same every time, except for the thetas, 
which are a representatoin of the current rotation?

"""

import DH_link
import numpy as np
from math import pi
from math import radians, asin, cos, atan2

class IRB4400_DH:

    """
    CONSTANT - IRB4400 DH parameters - DO NOT MODIFY UNLESS SURE 
    -Dynamic_Analysis_and_Visualization_of_Spatial_Mani suggests modifications to theta values
    These determine all the maths for the forward/inverse kinematics - if incorrect there will be unpredictable behaviour   
    Format is [link_length, link twist, link offset] 
     or       [a,             alpha,        d]
     These are constant for each joint - hence why theta is in a different list

    When IRB440 is at the position (thetas = 0, 0, 0, 0, 0, 0)
    The coords are registered as [X,Y,Z] = [1960, 0, 2075] in reference to the base frame

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
    REMEMBER THAT A6 WILL NEED UPDATING DEPENDENT ON THE END AFFECTOR
    """

    _DH_PARAMS = [[240, pi/2, 800], [1050, 0, 0], [225, pi/2, 0], [0, -pi/2, 1520], [0,pi/2,0], [0,0,200]]







    #Should be the same everytime
    def __init__(self, theta_list):

        if len(theta_list) != 6:
            print("WARNING: THETA LIST INCORRECT LENGTH")

        self.theta_list = theta_list




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
        
        self.T = np.matmul(
                    np.matmul(np.matmul(self.link_list[0].get_hg_mat(), self.link_list[1].get_hg_mat()), 
                        np.matmul(self.link_list[2].get_hg_mat(), self.link_list[3].get_hg_mat())),  
                        np.matmul(self.link_list[4].get_hg_mat(), self.link_list[5].get_hg_mat()))
        

        

    """
    Calculate the transform and extract the position and the orientation data
    """
    def update_pos_orient(self):
        #Calculate the trasnform matrix
        self._calc_transform()

        #Extract the position and the orientation from the transform matrix - according to M.Spong
        self.pos = {

            "X": self.T[0][3], 
            "Y": self.T[1][3], 
            "Z": self.T[2][3]
        }

        self.pure_orient_mat = [[self.T[0][0], self.T[0][1], self.T[0][2]], [self.T[1][0], self.T[1][1], self.T[1][2]], [self.T[2][0], self.T[2][1], self.T[2][2]]]



        
    """
    Updates the joint angles
    """
    def update_joint_angles(self, theta_list):
        
        if len(theta_list) != 6:
            print("WARNING: THETA LIST INCORRECT LENGTH")

        self.theta_list = theta_list


        self._THETA_OFFSETS = [0, pi/2, -self.theta_list[1], 0, 0, 0]

        for i in range(len(theta_list)):
            self.link_list[i].update_hg_mat(joint_angle = theta_list[i] + self._THETA_OFFSETS[i])

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

            ang_1 = -asin(self.pure_orient_mat[2][0])

            ang_2 = pi - ang_1


            #All inverted to match RobotStudio (also inverted R11 in the phi calculation)
            self.euler_orient =[
                [-atan2(self.pure_orient_mat[2][1]/cos(ang_1), self.pure_orient_mat[2][2]/cos(ang_1)), -ang_1, 
                 -atan2(self.pure_orient_mat[1][0]/cos(ang_1), -self.pure_orient_mat[0][0]/cos(ang_1))],
                [-atan2(self.pure_orient_mat[2][1]/cos(ang_2), self.pure_orient_mat[2][2]/cos(ang_2)), -ang_2, 
                 -atan2(self.pure_orient_mat[1][0]/cos(ang_2), -self.pure_orient_mat[0][0]/cos(ang_2))]
                ]

        else:

            #There are infinite different solutions at this point!
            #We constrain phi to be 0 as we only want one solution
            
            """
            NOTE: In the future it might be important to read the angle/joint data from the robot as opposed to calculating it ourselves because
                  we might end up constraining phi to be something that it isnt! (although technically we will have the same outcome)
            """
            phi = 0

            if(self.pure_orient_mat[2][0] == -1):
                theta = pi/2
                psi = phi + atan2(self.pure_orient_mat[0][1], self.pure_orient_mat[0][2])

            else:
                theta = -pi/2
                psi = -phi + atan2(-self.pure_orient_mat[0][1], -self.pure_orient_mat[0][2])


            #All inverted to match robot studio
            self.euler_orient[-psi, -theta, -phi]



        return self.euler_orient


if __name__ == "__main__":


    theta_1 = radians(-67.75) #NON-INVERTED
    theta_2 = -radians(58.01) #INVERTED
    theta_3 = -radians(8.05) #INVERTED
    theta_4 = radians(195.51) #NON-INVERTED
    theta_5 = -radians(43.82) # INVERTED
    theta_6 = radians(-94.38) # NON-INVERTED!

    robot = IRB4400_DH([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6])

    robot.update_pos_orient()


    pos = robot.get_pos()

    X = round(float(pos["X"]), 3)
    Y = round(float(pos["Y"]), 3)
    Z = round(float(pos["Z"]), 3)


    print(f"X: {X}  Y: {Y}  Z: {Z}")
    print(robot.get_euler_orient())







