"""
The DH representaiton of the 6-axis robot

The DH representation (or at least the lengths/angles) should be the same every time, except for the thetas, 
which are a representatoin of the current rotation?

"""

import DH_link
import numpy as np
from math import pi
from math import radians

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
        self.pos = [self.T[0][3], self.T[1][3], self.T[2][3]]

        self.orient = [[self.T[0][0], self.T[0][1], self.T[0][2]], [self.T[1][0], self.T[1][1], self.T[1][2]], [self.T[2][0], self.T[2][1], self.T[2][2]]]

        
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
    def get_orient(self):
        return self.orient


if __name__ == "__main__":


    theta_1 = radians(-45) #NON-INVERTED
    theta_2 = radians(25) #INVERTED
    theta_3 = radians(15) #INVERTED
    theta_4 = radians(50) #NON-INVERTED
    theta_5 = -radians(30) # INVERTED
    theta_6 = radians(-264.61) # NON-INVERTED!

    robot = IRB4400_DH([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6])

    robot.update_pos_orient()


    pos = robot.get_pos()

    X = round(float(pos[0]), 6)
    Y = round(float(pos[1]), 6)
    Z = round(float(pos[2]), 6)


    print(f"X: {X}  Y: {Y}  Z: {Z}")
    print(robot.get_orient())







