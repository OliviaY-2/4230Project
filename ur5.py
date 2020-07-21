#Class definition in python for ur5 robot
#Written by Olivia Yem
#class to allow the calculation of kinematics for the ur5 robot

import numpy as np

class ur5{
	#member attributes
	def _init_():
		self.theta1 = 0; #the current position of each joint
		self.theta2 = 0;
		self.theta3 = 0;
		self.theta4 = 0;
		self.theta5 = 0;
		self.theta6 = 0;
		self.J = np.zeros((6,6)); #the Jacobian of the robot
		self.T = np.zeros((6,6)); #0T6 transformation matrix

	#instance methods
	#setters
	def setTheta1(double angle):
		self.theta1 = angle
	def setTheta2(double angle):
		self.theta2 = angle
	def setTheta3(double angle):
		self.theta3 = angle
	def setTheta4(double angle):
		self.theta4 = angle
	def setTheta5(double angle):
		self.theta5 = angle
	def setTheta6(double angle):
		self.theta6 = angle

	#define member attributes from others
	def calcT(): #calculates the current value of T using the thetas
		pass
	def calcJ(): #calculates the current value of J using the thetas
		pass

	#getters
	def getTheta1():
		return self.theta1
	def getTheta2():
		return self.theta2
	def getTheta3():
		return self.theta3
	def getTheta4():
		return self.theta4
	def getTheta5():
		return self.theta5
	def getTheta6():
		return self.theta6

	#kinematics
	def forwardKin(): #return the position of the end effector with the current joint positions
		calcT();
		p6 = np.array[[0][0][0][1]]	
		p0 = np.matmul(T,p6)
		p = p0[:3]
		return p
	def inverseKin(double x, double y, double z): #return the joint positions for the target position x,y,z
		pass
}