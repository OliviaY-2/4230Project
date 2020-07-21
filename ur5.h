//Class Header for ur5 Robot
//Written by Olivia Yem
//Written to enable kinematics

#ifndef UR5_H
#define UR5_H

class ur5{
	double theta1, theta2, theta3, theta4, theta5, theta6; //the current position of each joint
	double J[6][6]; //the Jacobian of the robot
	double T[6][6]; //0T6 transformation matrix
public:
	ur5();
	~ur5();
	void setTheta1(double angle);
	void setTheta2(double angle);
	void setTheta3(double angle);
	void setTheta4(double angle);
	void setTheta5(double angle);
	void setTheta6(double angle);
	double getTheta1();
	double getTheta2();
	double getTheta3();
	double getTheta4();
	double getTheta5();
	double getTheta6();
	double* forwardKin(); //return the position of the end effector with the current joint positions
	double* inverseKin(double x, double y, double z); //return the joint positions for the target position x,y,z
}

#endif