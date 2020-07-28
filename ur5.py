#Class definition in python for ur5 robot
#Written by Olivia Yem
#class to allow the calculation of kinematics for the ur5 robot

import numpy as np
import math as m

class ur5Robot:
	#member attributes
	def __init__(self):
		self.theta1 = 0; #the current position of each joint
		self.theta2 = 0;
		self.theta3 = 0;
		self.theta4 = 0;
		self.theta5 = 0;
		self.theta6 = 0;
		self.T = np.zeros((6,6)); #0T6 transformation matrix
		self.d1 = 0.089159 #constants of geometry
		self.a1 = 0
		self.d2 = 0
		self.a2 = -0.425
		self.d3 = 0
		self.a3 = -0.39225
		self.d4 = 0.10915
		self.a4 = 0
		self.d5 = 0.09465
		self.a5 = 0
		self.d6 = 0.0823
		self.a6 = 0
	#
	#instance methods
	#setters
	def setTheta1(self,angle):
		self.theta1 = angle
	def setTheta2(self,angle):
		self.theta2 = angle
	def setTheta3(self,angle):
		self.theta3 = angle
	def setTheta4(self,angle):
		self.theta4 = angle
	def setTheta5(selfmangle):
		self.theta5 = angle
	def setTheta6(self,angle):
		self.theta6 = angle
	def setAllTheta(self,angle1, angle2, angle3, angle4, angle5, angle6):
		self.theta1 = angle1
		self.theta2 = angle2
		self.theta3 = angle3
		self.theta4 = angle4
		self.theta5 = angle5
		self.theta6 = angle6
	#
	#define member attributes from others
	def calcT(self): #calculates the current value of T using the thetas
		nx = m.cos(self.theta6)*(m.sin(self.theta1)*m.sin(self.theta5)+((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0+((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0)-(m.sin(self.theta6)*((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))-(m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))))/2.0
		ny = m.cos(self.theta6)*(((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.cos(self.theta1)*m.sin(self.theta2+self.theta4+self.theta3))*m.cos(self.theta5))/2.0-m.cos(self.theta1)*m.sin(self.theta5)+((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0)-m.sin(self.theta6)*((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))/2.0-(m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))/2.0)
		nz = (m.sin(self.theta2+self.theta3+self.theta4)*m.cos(self.theta6)+m.cos(self.theta2+self.theta3+self.theta4)*m.sin(self.theta6))/2.0+m.sin(self.theta2+self.theta3+self.theta4)*m.cos(self.theta5)*m.cos(self.theta6)-(m.sin(self.theta2+self.theta3+self.theta4)*m.cos(self.theta6)-m.cos(self.theta2+self.theta3+self.theta4)*m.sin(self.theta6))/2.0
		ox = -1*(m.cos(self.theta6)*((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))-(m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))))/2.0-m.sin(self.theta6)*(m.sin(self.theta1)*m.sin(self.theta5)+((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0+((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0)
		oy = m.cos(self.theta6)*((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))/2.0-(m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))/2.0)-m.sin(self.theta6)*(((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0-m.cos(self.theta1)*m.sin(self.theta5)+((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.cos(self.theta5))/2.0)
		oz = (m.cos(self.theta2+self.theta3+self.theta4)*m.cos(self.theta6)+m.sin(self.theta2+self.theta3+self.theta4)*m.sin(self.theta6))/2.0+(m.cos(self.theta2+self.theta3+self.theta4)*m.cos(self.theta6)-m.sin(self.theta2+self.theta3+self.theta4)*m.sin(self.theta6))/2.0-m.sin(self.theta2+self.theta3+self.theta4)*m.cos(self.theta5)*m.sin(self.theta6)
		ax = m.cos(self.theta5)*m.sin(self.theta1)-((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0-((m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0
		ay = -1*m.cos(self.theta1)*m.cos(self.theta5)-((m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0+((m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0
		az = (m.cos(self.theta2+self.theta3+self.theta4)*m.cos(self.theta5)-m.sin(self.theta2+self.theta3+self.theta4)*m.sin(self.theta5))/2.0-(m.cos(self.theta2+self.theta3+self.theta4)*m.cos(self.theta5)+m.sin(self.theta2+self.theta3+self.theta4)*m.sin(self.theta5))/2.0
		px = -1*(self.d5*(m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4)))/2.0+(self.d5*(m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4)))/2.0+self.d4*m.sin(self.theta1)-(self.d6*(m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0-(self.d6*(m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)+m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0+self.a2*m.cos(self.theta1)*m.cos(self.theta2)+self.d6*m.cos(self.theta5)*m.sin(self.theta1)+self.a3*m.cos(self.theta1)*m.cos(self.theta2)*m.cos(self.theta3)-self.a3*m.cos(self.theta1)*m.sin(self.theta2)*m.sin(self.theta3)
		py = -1*(self.d5*(m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4)))/2.0+(self.d5*(m.cos(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)*m.sin(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4)))/2.0-self.d4*m.cos(self.theta1)-(self.d6*(m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0-(self.d6*(m.sin(self.theta1)*m.cos(self.theta2+self.theta3+self.theta4)-m.cos(self.theta1)*m.sin(self.theta2+self.theta3+self.theta4))*m.sin(self.theta5))/2.0-self.d6*m.cos(self.theta1)*m.cos(self.theta5)+self.a2*m.cos(self.theta2)*m.sin(self.theta1)+self.a3*m.cos(self.theta2)*m.cos(self.theta3)*m.sin(self.theta1)-self.a3*m.sin(self.theta1)*m.sin(self.theta2)*m.sin(self.theta4)
		pz = self.d1+(self.d6*(m.cos(self.theta2+self.theta3+self.theta4)*m.cos(self.theta5)-m.sin(self.theta2+self.theta3+self.theta4)+m.sin(self.theta5)))/2.0+self.a3*(m.sin(self.theta2)*m.cos(self.theta3)+m.cos(self.theta2)*m.sin(self.theta3))+self.a2*m.sin(self.theta2)-(self.d6*(m.cos(self.theta2+self.theta3+self.theta4)*m.cos(self.theta5)+m.sin(self.theta2+self.theta3+self.theta4)*m.sin(self.theta5)))/2.0-self.d5*m.cos(self.theta2+self.theta3+self.theta4)
		self.T = np.array([[nx, ox, ax, px],[ny, oy, ay, py],[nz, oz, az, pz],[0, 0, 0, 1]])
	#
	#getters
	def getTheta1(self):
		return self.theta1
	def getTheta2(self):
		return self.theta2
	def getTheta3(self):
		return self.theta3
	def getTheta4(self):
		return self.theta4
	def getTheta5(self):
		return self.theta5
	def getTheta6(self):
		return self.theta6
	#
	#kinematics
	def forwardKin(self): #return the position of the start of the end effector/end of link 6 with the current joint positions
		self.calcT();
		p = np.zeros((6,1))
		p6 = np.array([[0],[0],[0],[1]])	
		p0 = np.matmul(self.T,p6)
		p[:3] = p0[:3]
		p[3] = m.atan2(self.T[3,2],self.T[3,3])
		p[4] = m.atan2(self.T[3,1],m.sqrt(m.pow(self.T[3,2],2)+m.pow(self.T[3,3],2)))
		p[5] = m.atan2(self.T[2,1],self.T[1,1])
		return p
	def calcT14(self,T06,theta1,theta5,theta6):
		T01 = np.array([[m.cos(theta1),0,m.sin(theta1),0],[m.sin(theta1),m.cos(theta1),-1*m.cos(theta1),0],[0,1,0,self.d1],[0,0,0,1]])
		T56 = np.array([[m.cos(theta6),-1*m.sin(theta6),0,0],[m.sin(theta6),0,0,0],[0,0,1,self.d6],[0,0,0,1]])
		T45 = np.array([[m.cos(theta5),0,m.sin(theta5),0],[m.sin(theta5),m.cos(theta5),-1*m.cos(theta5),0],[0,1,0,self.d5],[0,0,0,1]])
		if np.linalg.det(T01)==0 or np.linalg.det(T45)==0 or np.linalg.det(T56)==0:
			T14 = np.zeros((4,4))
			T14[0:4,0:4]=np.nan
		else:
			T10 = np.linalg.inv(T01)
			T65 = np.linalg.inv(T56)
			T54 = np.linalg.inv(T45)
			T16 = np.matmul(T10,T06)
			T64 = np.matmul(T65,T54)
			T14 = np.matmul(T16,T64)
		return T14
	def inverseKin(self,x, y, z,roll,pitch,yaw): #return the joint positions for the end of link 6 with target position x,y,z and orientation roll=c,pitch=b,yaw =a
		#Code here to turn input arguments into a transformation matrix called target
		target = np.array([[m.cos(yaw)*m.cos(pitch), m.cos(yaw)*m.sin(pitch)*m.sin(roll)-m.sin(yaw)*m.cos(roll), m.cos(yaw)*m.sin(pitch)*m.cos(roll)+m.sin(yaw)*m.sin(roll), x],[m.sin(yaw)*m.cos(pitch), m.sin(yaw)*m.sin(pitch)*m.sin(yaw)+m.cos(yaw)*m.cos(roll), m.sin(yaw)*m.sin(pitch)*m.cos(roll)-m.cos(yaw)*m.sin(roll), y],[-1*m.sin(pitch), m.cos(pitch)*m.sin(roll), m.cos(pitch)*m.cos(roll), z],[0,0,0,1]]) #also 0T6
		thetas = np.zeros((6, 8)) #6x8 matrix, each colum represents one possible solution of joint angles
	#calculate theta1
		change = np.array([[0],[0],[-1*self.d6],[1]])
		p05 = np.matmul(target,change)
		thetas[0,0] = np.real(m.atan2(p05[2],p05[1])+m.acos(self.d4/m.sqrt(m.pow(p05[1],2)+m.pow(p05[2],2)))+m.pi/2)
		thetas[0,1] = thetas[0,0]
		thetas[0,2] = thetas[0,0]
		thetas[0,3] = thetas[0,0]
		thetas[0,4] = np.real(m.atan2(p05[2],p05[1])-m.acos(self.d4/m.sqrt(m.pow(p05[1],2)+m.pow(p05[2],2)))+m.pi/2)
		thetas[0,5] = thetas[0,4]
		thetas[0,6] = thetas[0,4]
		thetas[0,7] = thetas[0,4]
	#Calculate theta5
		if (x*m.sin(thetas[0,0])-y*m.cos(thetas[0,0])-self.d4)/self.d6 <= 1 and (x*m.sin(thetas[0,0])-y*m.cos(thetas[0,0])-self.d4)/self.d6 >= -1:
			thetas[4,0] = np.real(m.acos((x*m.sin(thetas[0,0])-y*m.cos(thetas[0,0])-self.d4)/self.d6))
			thetas[4,1] = thetas[4,0]
			thetas[4,2] = np.real(-1*m.acos((x*m.sin(thetas[0,0])-y*m.cos(thetas[0,0])-self.d4)/self.d6))
			thetas[4,3] = thetas[4,2]
		else:
			thetas[4,0] = np.nan
			thetas[4,1] = np.nan
			thetas[4,2] = np.nan
			thetas[4,3] = np.nan
		if (x*m.sin(thetas[0,4])-y*m.cos(thetas[0,4])-self.d4)/self.d6 <= 1 and (x*m.sin(thetas[0,4])-y*m.cos(thetas[0,4])-self.d4)/self.d6 >= -1:
			thetas[4,4] = np.real(m.acos((x*m.sin(thetas[0,4])-y*m.cos(thetas[0,4])-self.d4)/self.d6))
			thetas[4,5] = thetas[4,4]
			thetas[4,6] = np.real(-1*m.acos((x*m.sin(thetas[0,4])-y*m.cos(thetas[0,4])-self.d4)/self.d6))
			thetas[4,7] = thetas[4,6]
		else:
			thetas[4,4] = np.nan
			thetas[4,5] = np.nan
			thetas[4,6] = np.nan
			thetas[4,7] = np.nan
	#calculate theta6
		t60 = np.linalg.inv(target)
		for i in range(8):
			thetas[5,i] = np.real(m.atan2((-1*t60[0,1]*m.sin(thetas[0,i])+t60[1,1]*m.sin(thetas[0,i]))/m.sin(thetas[4,i]),(t60[0,0]*m.sin(thetas[0,i])-t60[0,1]*m.cos(thetas[0,i]))/m.sin(thetas[4,i])))
	#calculate theta3
		#columns 1,2
		T14 = self.calcT14(target,thetas[0,1],thetas[4,1],thetas[5,1])
		thetas[2,0] = np.real(m.acos(m.sqrt(pow(T14[0,3],2)+pow(T14[2,3],2))-pow(self.a2,2)-(pow(self.a3,2))/(2*self.a2*self.a3)))
		thetas[2,1] = -1*thetas[2,0]
		#columns 3,4
		T14 = self.calcT14(target,thetas[0,2],thetas[4,2],thetas[5,2])
		thetas[2,2] = np.real(m.acos(m.sqrt(pow(T14[0,3],2)+pow(T14[2,3],2))-pow(self.a2,2)-(pow(self.a3,2))/(2*self.a2*self.a3)))
		thetas[2,3] = -1*thetas[2,2]
		#columns 5,6
		T14 = self.calcT14(target,thetas[0,4],thetas[4,4],thetas[5,4])
		thetas[2,4] = np.real(m.acos(m.sqrt(pow(T14[0,3],2)+pow(T14[2,3],2))-pow(self.a2,2)-(pow(self.a3,2))/(2*self.a2*self.a3)))
		thetas[2,5] = -1*thetas[2,4]
		#columns 7,8
		T14 = self.calcT14(target,thetas[0,7],thetas[4,7],thetas[5,7])
		thetas[2,6] = np.real(m.acos(m.sqrt(pow(T14[0,3],2)+pow(T14[2,3],2))-pow(self.a2,2)-(pow(self.a3,2))/(2*self.a2*self.a3)))
		thetas[2,7] = -1*thetas[2,6]
	#calculate theta2 and theta4
		for i in range(8):
			T14 = self.calcT14(target,thetas[0,i],thetas[4,i],thetas[5,i])
			thetas[1,i] = np.real(m.atan2(-1*T14[2,3],-1*T14[0,3])-m.asin(-1*self.a3*m.sin(thetas[2,i])/m.sqrt(pow(T14[0,3],2)+pow(T14[2,3],2))))
			T12 = np.array([[m.cos(thetas[1,i]),-1*m.sin(thetas[1,i]),0,self.a2*m.cos(thetas[1,i])],[m.sin(thetas[1,i]),0,0,self.a3*m.sin(thetas[1,i])],[0,0,1,0],[0,0,0,1]])
			T23 = np.array([[m.cos(thetas[2,i]),-1*m.sin(thetas[2,i]),0,self.a3*m.cos(thetas[2,i])],[m.sin(thetas[2,i]),0,0,self.a3*m.sin(thetas[2,i])],[0,0,1,0],[0,0,0,1]])
			if np.linalg.det(T12)==0 or np.linalg.det(T23)==0:
				T34 = np.zeros((4,4))
				T34[0:4,0:4]=np.nan
			else:
				T21 = np.linalg.inv(T12)
				T32 = np.linalg.inv(T23)
				T31 = np.matmul(T32,T21)
				T34 = np.matmul(T31,T14)
			thetas[3,i] = m.atan2(T34[1,0],T34[0,0])
		return thetas
