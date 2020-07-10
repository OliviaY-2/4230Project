%MTRN4230 T2 2020
%Kinematics of the UR5 arm
%Written by Olivia Yem

%given the position and orientation of the end effector, the following
%function will return the possible angles of each joint

function [t1,t2,t3,t4,t5,t6] = inverseKin(x,y,z,gamma,psi)
%define the variables
syms theta1 theta2 theta3 theta4 theta5 theta6;
L = 0; %length of the gripper in mm THIS IS A PLACEHOLDER VALUE

%define the transformation matrices
gTn = [cos(psi)*cos(gamma), -1*cos(psi)*sin(gamma), sin(psi), x;
       sin(psi), cos(gamma), 0, y;
       -1*sin(psi)*cos(gamma), sin(psi)*sin(gamma), cos(psi), z;
       0, 0, 0, 1;];
   
Tn = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, L; 0, 0, 0, 1;];

T1 = [cos(theta1), -1*sin(theta1)*cos(pi/2), sin(theta1*sin(pi/2)), 0*cos(theta1);
      sin(theta1), cos(theta1*sin(pi/2)), -1*cos(theta1)*sin(pi/2), 0*sin(theta1);
      0, sin(pi/2), cos(pi/2), 89.2;
      0, 0, 0, 1;];
  
T2 = [cos(theta2), -1*sin(theta2)*cos(0), sin(theta2*sin(0)), 425*cos(theta2);
      sin(theta2), cos(theta2*sin(0)), -1*cos(theta2)*sin(0), 425*sin(theta2);
      0, sin(0), cos(0), 0;
      0, 0, 0, 1;];
T3 = [cos(theta3), -1*sin(theta3)*cos(0), sin(theta3*sin(0)), 392*cos(theta3);
      sin(theta3), cos(theta3*sin(0)), -1*cos(theta3)*sin(0), 392*sin(theta3);
      0, sin(0), cos(0), 0;
      0, 0, 0, 1;];
T4 = [cos(theta4), -1*sin(theta4)*cos(pi/2), sin(theta4*sin(pi/2)), 0*cos(theta4);
      sin(theta4), cos(theta4*sin(pi/2)), -1*cos(theta4)*sin(pi/2), 0*sin(theta4);
      0, sin(pi/2), cos(pi/2), 109.3;
      0, 0, 0, 1;];
T5 = [cos(theta5), -1*sin(theta5)*cos(pi/2), sin(theta5*sin(pi/2)), 0*cos(theta5);
      sin(theta5), cos(theta5*sin(pi/2)), -1*cos(theta5)*sin(pi/2), 0*sin(theta5);
      0, sin(pi/2), cos(pi/2), 94.7;
      0, 0, 0, 1;];
T6 = [cos(theta6), -1*sin(theta6)*cos(0), sin(theta6*sin(0)), 0*cos(theta6);
      sin(theta6), cos(theta6*sin(0)), -1*cos(theta6)*sin(0), 0*sin(theta6);
      0, sin(0), cos(0), 82.5;
      0, 0, 0, 1;];
    
eqn = gTn == T1*T2*T3*T4*T5*T6*Tn;
  
[t1,t2,t3,t4,t5,t6] = solve(eqn,[theta1 theta2 theta3 theta4 theta5 theta6]);
end