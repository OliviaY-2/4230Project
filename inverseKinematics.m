%MTRN4230 T2 2020
%Kinematics of the UR5 arm
%Written by Olivia Yem

%given the position and orientation of the end effector, the following
%function will return the possible angles of each joint

% function inverseKinematics
% %define the variables
% syms theta1 theta2 theta3 theta4 theta5 theta6;
% L = 0; %length of the gripper in mm THIS IS A PLACEHOLDER VALUE
% 
% %define the transformation matrices
% t01 = [cos(theta1), -1*sin(theta1)*cos(pi/2), sin(theta1)*sin(pi/2), 0*cos(theta1);
%       sin(theta1), cos(theta1*sin(pi/2)), -1*cos(theta1)*sin(pi/2), 0*sin(theta1);
%       0, sin(pi/2), cos(pi/2), 89.2;
%       0, 0, 0, 1;];
%   
% t12 = [cos(theta2), -1*sin(theta2)*cos(0), sin(theta2)*sin(0), 425*cos(theta2);
%       sin(theta2), cos(theta2*sin(0)), -1*cos(theta2)*sin(0), 425*sin(theta2);
%       0, sin(0), cos(0), 0;
%       0, 0, 0, 1;];
% t23 = [cos(theta3), -1*sin(theta3)*cos(0), sin(theta3)*sin(0), 392*cos(theta3);
%       sin(theta3), cos(theta3*sin(0)), -1*cos(theta3)*sin(0), 392*sin(theta3);
%       0, sin(0), cos(0), 0;
%       0, 0, 0, 1;];
% t34 = [cos(theta4), -1*sin(theta4)*cos(pi/2), sin(theta4)*sin(pi/2), 0*cos(theta4);
%       sin(theta4), cos(theta4*sin(pi/2)), -1*cos(theta4)*sin(pi/2), 0*sin(theta4);
%       0, sin(pi/2), cos(pi/2), 109.3;
%       0, 0, 0, 1;];
% t45 = [cos(theta5), -1*sin(theta5)*cos(pi/2), sin(theta5)*sin(pi/2), 0*cos(theta5);
%       sin(theta5), cos(theta5*sin(pi/2)), -1*cos(theta5)*sin(pi/2), 0*sin(theta5);
%       0, sin(pi/2), cos(pi/2), 94.7;
%       0, 0, 0, 1;];
% t56 = [cos(theta6), -1*sin(theta6)*cos(0), sin(theta6)*sin(0), 0*cos(theta6);
%       sin(theta6), cos(theta6*sin(0)), -1*cos(theta6)*sin(0), 0*sin(theta6);
%       0, sin(0), cos(0), 82.5;
%       0, 0, 0, 1;];
% t67 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, L; 0, 0, 0, 1;];
% 
% %calculate the base to frame T matrices
% t02 = t01*t12;
% t03 = t01*t12*t23;
% t04 = t01*t12*t23*t34;
% t05 = t01*t12*t23*t34*t45;
% t06 = t01*t12*t23*t34*t45*t56;
% t07 = t01*t12*t23*t34*t45*t56*t67;
% 
% %define the jacobian
% 
% end

%inputs are the current positions of the joints and the target position
%assumes global varible J for the Jacobean
%dTheta = J^T * dPosition
function [theta] = InverseKin(x,y,z,ct1,ct2,ct3,ct4,ct5,ct6) 
    syms theta1 theta2 theta3 theta4 theta5 theta6;
    currentPos = forwardKin([ct1,ct2,ct3,ct4,ct5,ct6]);
    V = [x;y;z] - currentPos; %the required change in position
    numJ = 
    dTheta = 
end