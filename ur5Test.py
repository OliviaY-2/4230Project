#Test of kinematics in python for ur5 robot
#Written by Olivia Yem

from ur5 import ur5Robot
import math as m
import numpy as np

robot = ur5Robot()
print("TEST 1 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
robot.setAllTheta(2.08, 1.78, 0.02, 0.66, 1.83, 2.98)
print("Theta1 is",robot.getTheta1())
print("Theta2 is",robot.getTheta2())
print("Theta3 is",robot.getTheta3())
print("Theta4 is",robot.getTheta4())
print("Theta5 is",robot.getTheta5())
print("Theta6 is",robot.getTheta6())
p = robot.forwardKin()
print("The end effector is at", p)
thetas = robot.inverseKin(p[0],p[1],p[2],p[3],p[4],p[5])
print("The possible values of the joint positions are:\n",thetas)
