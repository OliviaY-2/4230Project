#Class definition in python for ur5 robot
#Written by Olivia Yem
#class to allow the calculation of kinematics for the ur5 robot

import numpy as np
import math as m

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
		d1 = 0.089159
		a2 = -0.425
		a3 = -0.39225
		d4 = 0.10915
		d5 = 0.09465
		d6 = 0.0823
		nx = m.cos(self.theta6)*(m.sin(self.theta1)*m.sin(self.theta5)+((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0+((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0)-(m.sin(self.theta6)*((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))-(m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))))/2.0
		ny = m.cos(self.theta6)*(((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.cos(self.theta1)*m.sin(self.theta2+self.theta4+self.theta3))*m.cos(self.theta5))/2.0-m.cos(self.theta1)*m.sin(self.theta5)+((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0)-m.sin(self.theta6)*((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))/2.0-(m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))/2.0)
		nz = (m.sin(self.theta2+self.theta3+self.theta4)*m.cos(self.theta6)+m.cos(self.theta2+self.theta3+self.theta4)*m.sin(self.theta6))/2.0+m.sin(self.theta2+self.theta3+self.theta4)*m.cos(self.theta5)*m.cos(self.theta6)-(m.sin(self.theta2+self.theta3+self.theta4)*m.cos(self.theta6)-m.cos(self.theta2+self.theta3+self.theta4)*m.sin(self.theta6))/2.0
		ox = -1*(m.cos(self.theta6)*((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))-(m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))))/2.0-m.sin(self.theta6)*(m.sin(self.theta1)*m.sin(self.theta5)+((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0+((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.sin(self.theat1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0)
		oy = m.cos(self.theta6)*((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))/2.0-(m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))/2.0)-m.sin(self.theta6)*(((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0-m.cos(self.theta1)*m.sin(self.theta5)+((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0)
		oz = (m.cos(self.theta2+self.theta3+self.theta4)*m.cos(self.theta6)+m.sin(self.theta2+self.theta3+self.theta4)*m.sin(self.theta6))/2.0+(m.cos(self.theta2+self.theta3+self.theta4)*m.cos(self.theta6)-m.sin(self.theta2+self.theta3+self.theta4)*m.sin(self.theta6))/2.0-m.sin(self.theta2+self.theta3+self.theta4)*m.cos(self.theta5)*m.sin(self.theta6)
		ax = m.cos(self.theta5)*m.sin(self.theta1)-((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0-((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0
		ay = -1*m.cos(self.theta1)*m.cos(self.theta5)-((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0+((m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0
		az = (m.cos(self.theta2+self.theta3+self.theta4)*m.cos(self.theta5)-m.sin(self.theta2+self.theta3+self.theta4)*m.sin(self.theta5))/2.0-(m.cos(self.theta2+self.theta3+self.theta4)*m.cos(self.theta5)+m.sin(self.theta2+self.theta3+self.theta4)*m.sin(theta5))/2.0
		px = -1*(d5*(m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4)))/2.0+(d5*(m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4)))/2.0+d4*m.sin(self.theta1)-(d6*(m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0-(d6*(m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0+a2*m.cos(self.theta1)*m.cos(self.theta2)+d6*m.cos(self.theta5)*m.sin(self.theta1)+a3*m.cos(self.theta1)*m.cos(self.theta2)*m.cos(self.theta3)-a3*m.cos(self.theta1)*m.sin(self.theta2)*m.sin(self.theta3))
		py = -1*(d5*(m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4)))/2.0+(d5*(m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)*m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4)))/2.0-d4*m.cos(self.theta1)-(d6*(m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0-(d6*(m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0-d6*m.cos(self.theta1)*m.cos(self.theta5)+a2*m.cos(self.theta2)*m.sin(self.theta1)+a3*m.cos(self.theta2)*m.cos(self.theta3)*m.sin(self.theta1)-a3*m.sin(self.theta1)*m.sin(self.theta2)*m.sin(self.theta4))
		pz = d1+(d6*(m.cos(self.theta2+self.theta3+self.theta4)*m.cos(self.theta5)-m.sin(self.theta2+self.theta3+self.theta4)+m.sin(self.theta5)))/2.0+a3*(m.sin(self.theta2)*m.cos(self.theta3)+m.cos(self.theta2)*m.sin(self.theta3))+a2*m.sin(self.theta2)-(d6*(m.cos(self.theta2+self.theta3+self.theta4)*m.cos(self.theta5)+m.sin(self.theta2+self.theta3+self.theta4)*m.sin(self.theta5)))/2.0-d5*m.cos(self.theta2+self.theta3+self.theta4)
		T = [[nx ox ax px][ny oy ay py][nz oz az pz][0 0 0 1]]
		return T
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
	def forwardKin(): #return the position of the start of the end effector/end of link 6 with the current joint positions
		calcT();
		p6 = np.array[[0][0][0][1]]	
		p0 = np.matmul(T,p6)
		p = p0[:3]
		return p
	def inverseKin(double x, double y, double z): #return the joint positions for the end of link 6 with target position x,y,z and orientation roll,pitch,yaw 

}