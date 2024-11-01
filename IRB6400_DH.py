"""
The DH representaiton of the 6-axis robot

The DH representation (or at least the lengths/angles) should be the same every time, except for the thetas, 
which are a representatoin of the current rotation?

"""

import DH_link
import numpy as np
from math import pi

class IRB4400_DH:

    """
    CONSTANT - IRB4400 DH parameters - DO NOT MODIFY UNLESS SURE 
    -Dynamic_Analysis_and_Visualization_of_Spatial_Mani suggests modifications to theta values
    These determine all the maths for the forward/inverse kinematics - if incorrect there will be unpredictable behaviour   
    Format is [link_length, link twist, link offset] 
    """
    _DH_PARAMS = [[0.24, -pi/2, 0], [1.05, 0, 0], [0.225, -pi/2, 1.52], [0, pi/2, 0], [0, pi/2, 0], [0.4, 0, 0]]
    _THETA_OFFSETS = [-pi/2, 0, 0, -pi, 0, -pi/2]
    


    #Should be the same everytime
    def __init__(self, theta_list):

        if len(theta_list) != 6:
            print("WARNING: THETA LIST INCORRECT LENGTH")

        self.theta_list = theta_list




        self.link_list = []


        """
        TODO:
            -Generate constants to represent IRB4400 DH parameters
        """

        #Create all of the links with the relevant information
        for i in range(len(theta_list)):
            self.link_list.append(DH_link.DH_link(self._DH_PARAMS[i][0], 
                                                  self._DH_PARAMS[i][1], 
                                                  self._DH_PARAMS[i][2], 
                                                  self.theta_list[i] + self._THETA_OFFSETS[i])
                                                  )

        #Calc the current transformation matrix
        self.update_pos_orient()



    """
    Multiply the homogeneous matrices together to determine the transofmration matrix
    """
    def _calc_transform(self):


        #Multiply all of the homogeneus matrices together to get the transformation matrix of oritentation.translation frame 6 in respect to frame 0 (the reference frame)
        self.T = np.matmul(np.matmul(np.matmul(self.link_list[0].get_hg_mat(), self.link_list[1].get_hg_mat()), 
                  np.matmul(self.link_list[2].get_hg_mat(), self.link_list[3].get_hg_mat())),  
                  np.matmul(self.link_list[4].get_hg_mat(), self.link_list[5].get_hg_mat()))
        

        

    """
    Calculate the transform and extract the position and the orientation data
    """
    def update_pos_orient(self):
        #Calculate the trasnform matrix
        self._calc_transform()

        #Extract the position and the orientation from the transform matrix - according to M.Spong
        self.pos = [self.T[0][3], self.T[1][3], self.T[2][3]]

        self.orient = [[self.T[0][0], self.T[0][1], self.T[0][2]], [self.T[1][0], self.T[1][1], self.T[1][2]], [self.T[2][0], self.T[2][1], self.T[2][2]]]

        
    """
    Updates the joint angles
    """
    def update_joint_angles(self, theta_list):
        
        if len(theta_list) != 6:
            print("WARNING: THETA LIST INCORRECT LENGTH")

        self.theta_list = theta_list


        for i in range(len(theta_list)):
            self.link_list[i].update_hg_mat("joint_angle", theta_list[i] + self._THETA_OFFSETS[i])

        self.update_pos_orient()



    """
    Getter function for the position of the end affector relative to the base frame
    """
    def get_pos(self):
        return self.pos
    
    """
    Getter function for the orientation of the end affector relative to the base frame
    """
    def get_orient(self):
        return self.orient


if __name__ == "__main__":


    theta_1 = 0
    theta_2 = 0
    theta_3 = 0
    theta_4 = 0
    theta_5 = 30
    theta_6 = 0

    robot = IRB4400_DH([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6])

    robot.update_pos_orient()

    print(robot.get_pos())







