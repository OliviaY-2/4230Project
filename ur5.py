#Class definition in python for ur5 robot
#Written by Olivia Yem
#class to allow the calculation of kinematics for the ur5 robot

import numpy as np
import math as m

class ur5:
	#member attributes
	def _init_():
		self.theta1 = 0; #the current position of each joint
		self.theta2 = 0;
		self.theta3 = 0;
		self.theta4 = 0;
		self.theta5 = 0;
		self.theta6 = 0;
		self.T = np.zeros((6,6)); #0T6 transformation matrix
		d1 = 0.089159 #constants of geometry
		a1 = 0
		d2 = 0
		a2 = -0.425
		d3 = 0
		a3 = -0.39225
		d4 = 0.10915
		a4 = 0
		d5 = 0.09465
		a5 = 0
		d6 = 0.0823
		a6 = 0
	#
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
	#
	#define member attributes from others
	def calcT(): #calculates the current value of T using the thetas
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
	#
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
	#
	#kinematics
	def forwardKin(): #return the position of the start of the end effector/end of link 6 with the current joint positions
		calcT();
		p6 = np.array[[0][0][0][1]]	
		p0 = np.matmul(T,p6)
		p = p0[:3]
		return p
	def inverseKin(double x, double y, double z,roll,pitch,yaw): #return the joint positions for the end of link 6 with target position x,y,z and orientation roll,pitch,yaw 
		#Code here to turn input arguments into a transformation matrix called target
		target = np.zeros((4,4)) #also 0T6
		thetas = zeros(6, 8) #6x8 matrix, each colum represents one possible solution of joint angles
	#calculate theta1
		p05 = target-np.array([0][0][-1*d6][1])
		thetas(1,1:4) = np.real(m.atan2(p05(2),p05(1))+m.acos(d4/sqrt(pow(p05(1),2)+pow(p05(2),2)))+m.pi/2)
		thetas(1,5:8) = np.real(m.atan2(p05(2),p05(1))-m.acos(d4/sqrt(pow(p05(1),2)+pow(p05(2),2)))+m.pi/2)
	#Calculate theta5
		thetas(5,1:2) = np.real(m.acos((x*m.sin(thetas(1,1))-y*m.cos(thetas(1,1))-d4)/d6))
		thetas(5,3:4) = np.real(-1*m.acos((x*m.sin(thetas(1,1))-y*m.cos(thetas(1,1))-d4)/d6))
		thetas(5,5:6) = np.real(m.acos((x*m.sin(thetas(1,5))-y*m.cos(thetas(1,5))-d4)/d6))
		thetas(5,7:8) = np.real(-1*m.acos((x*m.sin(thetas(1,5))-y*m.cos(thetas(1,5))-d4)/d6))
	#calculate theta6
		t60 = np.linalg.inv(target)
		for i in range(8):
			thetas(6,i) = np.real(m.atan2((-1*t60(2,1)*m.sin(thetas(1,i))+t60(2,2)*m.sin(thetas(1,i)))/m.sin(thetas(5,i)),(t60(1,1)*m.sin(thetas(1,i))-t60(1,2)*m.cos(thetas(1,i)))/m.sin(thetas(5,i))))
	#calculate theta3
		#columns 1,2
		T10 = np.linalg.inv([[m.cos(thetas(1,1)) -1*m.sin(thetas(1,1))*m.cos(m.pi/2) m.sin(thetas(1,1)*m.sin(m.pi/2)) a1*m.cos(thetas(1,1))][m.sin(thetas(1,1)) m.cos(thetas(1,1)*m.sin(m.pi/2)) -1*m.cos(thetas(1,1))*m.sin(m.pi/2) a1*m.sin(thetas(1,1))][0 m.sin(m.pi/2) m.cos(m.pi/2) d1][0 0 0 1]])
		T65 = np.linalg.inv([[m.cos(thetas(6,1)) -1*m.sin(thetas(6,1))*m.cos(0) m.sin(thetas(6,1)*m.sin(0)) a6*m.cos(thetas(6,1))][m.sin(thetas(6,1)) m.cos(thetas(6,1)*sin(0)) -1*m.cos(thetas(6,1))*sin(0) a6*m.sin(thetas(6,1))][0 sin(0) cos(0) d6][0 0 0 1]])
		T54 = np.linalg.inv([[m.cos(thetas(5,1)) -1*m.sin(thetas(5,1))*m.cos(m.pi/2) m.sin(thetas(5,1)*m.sin(m.pi/2)) a5*m.cos(thetas(5,1))][m.sin(thetas(5,1)) m.cos(thetas(5,1)*m.sin(m.pi/2)) -1*m.cos(thetas(5,1))*m.sin(m.pi/2) a5*m.sin(thetas(5,1))][0 m.sin(m.pi/2) m.cos(m.pi/2) d5][0 0 0 1]])
		T16 = matmul(T10,target)
		T64 = matmul(T65,T54)
		T14 = matmul(T16,T64)
		thetas(3,1) = np.real(m.acos(m.sqrt(pow(T14(1,4),2)+pow(T14(3,4),2))-pow(a2,2)-(pow(a3,2))/(2*a2*a3)))
		thetas(3,2) = -1*thetas(3,1)
		#columns 3,4
		T10 = np.linalg.inv([[m.cos(thetas(1,3)) -1*m.sin(thetas(1,3))*m.cos(m.pi/2) m.sin(thetas(1,3)*m.sin(m.pi/2)) a1*m.cos(thetas(1,3))][m.sin(thetas(1,3)) m.cos(thetas(1,3)*m.sin(m.pi/2)) -1*m.cos(thetas(1,3))*m.sin(m.pi/2) a1*m.sin(thetas(1,3))][0 m.sin(m.pi/2) m.cos(m.pi/2) d1][0 0 0 1]])
		T65 = np.linalg.inv([[m.cos(thetas(6,3)) -1*m.sin(thetas(6,3))*m.cos(0) m.sin(thetas(6,3)*m.sin(0)) a6*m.cos(thetas(6,3))][m.sin(thetas(6,3)) m.cos(thetas(6,3)*sin(0)) -1*m.cos(thetas(6,3))*sin(0) a6*m.sin(thetas(6,3))][0 sin(0) cos(0) d6][0 0 0 1]])
		T54 = np.linalg.inv([[m.cos(thetas(5,3)) -1*m.sin(thetas(5,3))*m.cos(m.pi/2) m.sin(thetas(5,3)*m.sin(m.pi/2)) a5*m.cos(thetas(5,3))][m.sin(thetas(5,3)) m.cos(thetas(5,3)*m.sin(m.pi/2)) -1*m.cos(thetas(5,3))*m.sin(m.pi/2) a5*m.sin(thetas(5,3))][0 m.sin(m.pi/2) m.cos(m.pi/2) d5][0 0 0 1]])
		T16 = matmul(T10,target)
		T64 = matmul(T65,T54)
		T14 = matmul(T16,T64)
		thetas(3,3) = np.real(m.acos(m.sqrt(pow(T14(1,4),2)+pow(T14(3,4),2))-pow(a2,2)-(pow(a3,2))/(2*a2*a3)))
		thetas(3,4) = -1*thetas(3,3)
		#columns 5,6
		T10 = np.linalg.inv([[m.cos(thetas(1,5)) -1*m.sin(thetas(1,5))*m.cos(m.pi/2) m.sin(thetas(1,5)*m.sin(m.pi/2)) a1*m.cos(thetas(1,5))][m.sin(thetas(1,5)) m.cos(thetas(1,5)*m.sin(m.pi/2)) -1*m.cos(thetas(1,5))*m.sin(m.pi/2) a1*m.sin(thetas(1,5))][0 m.sin(m.pi/2) m.cos(m.pi/2) d1][0 0 0 1]])
		T65 = np.linalg.inv([[m.cos(thetas(6,5)) -1*m.sin(thetas(6,5))*m.cos(0) m.sin(thetas(6,5)*m.sin(0)) a6*m.cos(thetas(6,5))][m.sin(thetas(6,5)) m.cos(thetas(6,5)*sin(0)) -1*m.cos(thetas(6,5))*sin(0) a6*m.sin(thetas(6,5))][0 sin(0) cos(0) d6][0 0 0 1]])
		T54 = np.linalg.inv([[m.cos(thetas(5,5)) -1*m.sin(thetas(5,5))*m.cos(m.pi/2) m.sin(thetas(5,5)*m.sin(m.pi/2)) a5*m.cos(thetas(5,5))][m.sin(thetas(5,5)) m.cos(thetas(5,5)*m.sin(m.pi/2)) -1*m.cos(thetas(5,5))*m.sin(m.pi/2) a5*m.sin(thetas(5,5))][0 m.sin(m.pi/2) m.cos(m.pi/2) d5][0 0 0 1]])
		T16 = matmul(T10,target)
		T64 = matmul(T65,T54)
		T14 = matmul(T16,T64)
		thetas(3,5) = np.real(m.acos(m.sqrt(pow(T14(1,4),2)+pow(T14(3,4),2))-pow(a2,2)-(pow(a3,2))/(2*a2*a3)))
		thetas(3,6) = -1*thetas(3,5)
		#columns 7,8
		T10 = np.linalg.inv([[m.cos(thetas(1,7)) -1*m.sin(thetas(1,7))*m.cos(m.pi/2) m.sin(thetas(1,7)*m.sin(m.pi/2)) a1*m.cos(thetas(1,7))][m.sin(thetas(1,7)) m.cos(thetas(1,7)*m.sin(m.pi/2)) -1*m.cos(thetas(1,7))*m.sin(m.pi/2) a1*m.sin(thetas(1,7))][0 m.sin(m.pi/2) m.cos(m.pi/2) d1][0 0 0 1]])
		T65 = np.linalg.inv([[m.cos(thetas(6,7)) -1*m.sin(thetas(6,7))*m.cos(0) m.sin(thetas(6,7)*m.sin(0)) a6*m.cos(thetas(6,7))][m.sin(thetas(6,7)) m.cos(thetas(6,7)*sin(0)) -1*m.cos(thetas(6,7))*sin(0) a6*m.sin(thetas(6,7))][0 sin(0) cos(0) d6][0 0 0 1]])
		T54 = np.linalg.inv([[m.cos(thetas(5,7)) -1*m.sin(thetas(5,7))*m.cos(m.pi/2) m.sin(thetas(5,7)*m.sin(m.pi/2)) a5*m.cos(thetas(5,7))][m.sin(thetas(5,7)) m.cos(thetas(5,7)*m.sin(m.pi/2)) -1*m.cos(thetas(5,7))*m.sin(m.pi/2) a5*m.sin(thetas(5,7))][0 m.sin(m.pi/2) m.cos(m.pi/2) d5][0 0 0 1]])
		T16 = matmul(T10,target)
		T64 = matmul(T65,T54)
		T14 = matmul(T16,T64)
		thetas(3,7) = np.real(m.acos(m.sqrt(pow(T14(1,4),2)+pow(T14(3,4),2))-pow(a2,2)-(pow(a3,2))/(2*a2*a3)))
		thetas(3,8) = -1*thetas(3,8)
	#calculate theta2 and theta4
		for i in range(8)
			T10 = np.linalg.inv([[m.cos(thetas(1,i)) -1*m.sin(thetas(1,i))*m.cos(m.pi/2) m.sin(thetas(1,i)*m.sin(m.pi/2)) a1*m.cos(thetas(1,i))][m.sin(thetas(1,i)) m.cos(thetas(1,i)*m.sin(m.pi/2)) -1*m.cos(thetas(1,i))*m.sin(m.pi/2) a1*m.sin(thetas(1,i))][0 m.sin(m.pi/2) m.cos(m.pi/2) d1][0 0 0 1]])
			T65 = np.linalg.inv([[m.cos(thetas(6,i)) -1*m.sin(thetas(6,i))*m.cos(0) m.sin(thetas(6,i)*m.sin(0)) a6*m.cos(thetas(6,i))][m.sin(thetas(6,i)) m.cos(thetas(6,i)*sin(0)) -1*m.cos(thetas(6,i))*sin(0) a6*m.sin(thetas(6,i))][0 sin(0) cos(0) d6][0 0 0 1]])
			T54 = np.linalg.inv([[m.cos(thetas(5,i)) -1*m.sin(thetas(5,i))*m.cos(m.pi/2) m.sin(thetas(5,i)*m.sin(m.pi/2)) a5*m.cos(thetas(5,i))][m.sin(thetas(5,i)) m.cos(thetas(5,i)*m.sin(m.pi/2)) -1*m.cos(thetas(5,i))*m.sin(m.pi/2) a5*m.sin(thetas(5,i))][0 m.sin(m.pi/2) m.cos(m.pi/2) d5][0 0 0 1]])
			T16 = matmul(T10,target)
			T64 = matmul(T65,T54)
			T14 = matmul(T16,T64)
			thetas(2,i) = np.real(m.atan2(-1*T14(3,4),-1*T14(1,4))-m.asin(-1*a3*m.sin(thetas(3,i))/m.sqrt(pow(T14(1,4),2)+pow(T14(3,4),2))))
			T21 = np.linalg.inv([m.cos(thetas(2,i)), -1*m.sin(thetas(2,i))*m.cos(0), m.sin(thetas(2,i)*m.sin(0)) a2*m.cos(thetas(2,i))][m.sin(thetas(2,i)) m.cos(thetas(2,i)*m.sin(0)) -1*m.cos(thetas(2,i))*m.sin(0) a2*m.sin(thetas(2,i))][0 m.sin(0) m.cos(0) 0][0 0 0 1])
			T32 = np.linalg.inv([[m.cos(thetas(3,i)) -1*m.sin(thetas(3,i)) 0 a3*m.cos(thetas(3,i))][m.sin(thetas(3,i)) 0 0 a3*m.sin(thetas(3,1))][0 0 1 0][0 0 0 1]])
			T31 = matmul(T32,T21)
			T34 = matmul(T31,T14)
			thetas(4,i) = m.atan2(T34(2,1),T34(1,1))
	return thetas
