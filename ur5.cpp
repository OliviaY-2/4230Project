//Class file for ur5 Robot
//Written by Olivia Yem
//Written to enable kinematics

#include "ur5.h"

//constructor and destructor
ur5::ur5() {}
ur5::~ur5() {}

//setters
void ur5::setTheta1(double angle){
	theta1 = angle;
}
void ur5::setTheta2(double angle){
	theta2 = angle;
}
void ur5::setTheta3(double angle){
	theta3 = angle;
}
void ur5::setTheta4(double angle){
	theta4 = angle;
}
void ur5::setTheta5(double angle){
	theta5 = angle;
}
void ur5::setTheta6(double angle){
	theta6 = angle;
}

//getters
double ur5::getTheta1() {
	return theta1;
}
double ur5::getTheta2(){
	return theta2;
}
double ur5::getTheta3(){
	return theta3;
}
double ur5::getTheta4(){
	return theta4;
}
double ur5::getTheta5(){
	return theta5;
}
double ur5::getTheta6(){
	return theta6;
}

//kinematics
double* ur5::forwardKin(){
	static double p[3];
	
	return p;
}
