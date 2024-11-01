"""
Link class which represents a link in a DH chain.

Contains the information for the homogenous matrix


Standard A Matrix


      [cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), a_i*cos(theta_i)]
A_i = [sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), a_i*sin(theta_i)]
      [0,                      sin(alpha_i),             cos(alpha_i),            d_i       ]
      [0,                         0,                         0,                       1      ]

        a_i = link length
        alpha_i = link_twist
        d_i = link_offset
        theta_i = joint_angle


"""

import numpy as np
from math import sin, cos, pi

class DH_link:

    """
    These link parameters are with respect to the next joint
    I.e. z0 is the axis of actuation for joint 1

    """

    def __init__(self, link_length, link_twist, link_offset, joint_angle):
        
        #Assign the matrix variables
        self.a = link_length
        self.alpha = link_twist
        self.d = link_offset
        self.theta = joint_angle

        #Generate the matrix
        self.update_hg_mat()

        

    """
    Updates the homogenous matrix (the representation of the rotation and transformation in respect to the joint)
    Important note: Only theta should be changing during operation, the other are constants defining the frame relationships
    Consider changing the function so that it isn't possible to change the values other than theta - could speed up calculations?
    CURRENTLY IN RADIANS SO REMEMBER THAT
    """
    def update_hg_mat(self, **kwargs):

        self.a = kwargs.get("link_length", self.a)
        self.alpha = kwargs.get("link_twist", self.alpha)
        self.d = kwargs.get("link_offset", self.d)
        self.theta = kwargs.get("joint_angle", self.theta)


        #Sourced from M.Spong -Robot Modeling and Control
        self.A = np.array([[cos(self.theta), sin(self.theta)*cos(self.alpha), sin(self.theta)*sin(self.alpha), self.a*cos(self.theta)], 
                           [sin(self.theta), cos(self.theta)*cos(self.alpha), -cos(self.theta)*sin(self.alpha), self.a*sin(self.theta)],
                           [           0,           sin(self.alpha),                 cos(self.alpha),                 self.d          ], 
                           [           0,                     0,                           0,                         1               ]]
                           )


    def get_hg_mat(self):
        return self.A






if __name__ == "__main__":
    link = DH_link(1, pi/2, 1, pi)

    print(link.get_hg_mat())




    


