%MTRN4230 T2 2020
%Kinematics of the UR5 arm
%Written by Olivia Yem

%given the angle of each joint, the following function returns the position
%of the end effector in the global coordinate frame (p0)

function p = forwardKin(theta) %p0 is the position of the end effector, theta an array of joint positions in radians
%define variables 
L = 0; %length of the gripper in mm THIS IS A PLACEHOLDER VALUE
p6 = [0;0;L;1;]; %the origin of the gripper is at the origin of frame 6

%calculate the transformation matrices
T1 = [cos(theta(1)), -1*sin(theta(1))*cos(pi/2), sin(theta(1)*sin(pi/2)), 0*cos(theta(1));
      sin(theta(1)), cos(theta(1)*sin(pi/2)), -1*cos(theta(1))*sin(pi/2), 0*sin(theta(1));
      0, sin(pi/2), cos(pi/2), 89.2;
      0, 0, 0, 1;];
T2 = [cos(theta(2)), -1*sin(theta(2))*cos(0), sin(theta(2)*sin(0)), 425*cos(theta(2));
      sin(theta(2)), cos(theta(2)*sin(0)), -1*cos(theta(2))*sin(0), 425*sin(theta(2));
      0, sin(0), cos(0), 0;
      0, 0, 0, 1;];
T3 = [cos(theta(3)), -1*sin(theta(3))*cos(0), sin(theta(3)*sin(0)), 392*cos(theta(3));
      sin(theta(3)), cos(theta(3)*sin(0)), -1*cos(theta(3))*sin(0), 392*sin(theta(3));
      0, sin(0), cos(0), 0;
      0, 0, 0, 1;];
T4 = [cos(theta(4)), -1*sin(theta(4))*cos(pi/2), sin(theta(4)*sin(pi/2)), 0*cos(theta(4));
      sin(theta(4)), cos(theta(4)*sin(pi/2)), -1*cos(theta(4))*sin(pi/2), 0*sin(theta(4));
      0, sin(pi/2), cos(pi/2), 109.3;
      0, 0, 0, 1;];
T5 = [cos(theta(5)), -1*sin(theta(5))*cos(pi/2), sin(theta(5)*sin(pi/2)), 0*cos(theta(5));
      sin(theta(5)), cos(theta(5)*sin(pi/2)), -1*cos(theta(5))*sin(pi/2), 0*sin(theta(5));
      0, sin(pi/2), cos(pi/2), 94.7;
      0, 0, 0, 1;];
T6 = [cos(theta(6)), -1*sin(theta(6))*cos(0), sin(theta(6)*sin(0)), 0*cos(theta(6));
      sin(theta(6)), cos(theta(6)*sin(0)), -1*cos(theta(6))*sin(0), 0*sin(theta(6));
      0, sin(0), cos(0), 82.5;
      0, 0, 0, 1;];

  
%calculate end position
p0 = T1*T2*T3*T4*T5*T6*p6;
p = p0(1:3);
end
