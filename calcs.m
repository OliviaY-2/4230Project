
function J = defineJacobian()
%define the variables
syms theta1 theta2 theta3 theta4 theta5 theta6;
L = 0; %length of the gripper in mm THIS IS A PLACEHOLDER VALUE

%define the transformation matrices
t01 = [cos(theta1), -1*sin(theta1)*cos(pi/2), sin(theta1)*sin(pi/2), 0*cos(theta1);
      sin(theta1), cos(theta1*sin(pi/2)), -1*cos(theta1)*sin(pi/2), 0*sin(theta1);
      0, sin(pi/2), cos(pi/2), 89.2;
      0, 0, 0, 1;];
  
t12 = [cos(theta2), -1*sin(theta2)*cos(0), sin(theta2)*sin(0), 425*cos(theta2);
      sin(theta2), cos(theta2*sin(0)), -1*cos(theta2)*sin(0), 425*sin(theta2);
      0, sin(0), cos(0), 0;
      0, 0, 0, 1;];
t23 = [cos(theta3), -1*sin(theta3)*cos(0), sin(theta3)*sin(0), 392*cos(theta3);
      sin(theta3), cos(theta3*sin(0)), -1*cos(theta3)*sin(0), 392*sin(theta3);
      0, sin(0), cos(0), 0;
      0, 0, 0, 1;];
t34 = [cos(theta4), -1*sin(theta4)*cos(pi/2), sin(theta4)*sin(pi/2), 0*cos(theta4);
      sin(theta4), cos(theta4*sin(pi/2)), -1*cos(theta4)*sin(pi/2), 0*sin(theta4);
      0, sin(pi/2), cos(pi/2), 109.3;
      0, 0, 0, 1;];
t45 = [cos(theta5), -1*sin(theta5)*cos(pi/2), sin(theta5)*sin(pi/2), 0*cos(theta5);
      sin(theta5), cos(theta5*sin(pi/2)), -1*cos(theta5)*sin(pi/2), 0*sin(theta5);
      0, sin(pi/2), cos(pi/2), 94.7;
      0, 0, 0, 1;];
t56 = [cos(theta6), -1*sin(theta6)*cos(0), sin(theta6)*sin(0), 0*cos(theta6);
      sin(theta6), cos(theta6*sin(0)), -1*cos(theta6)*sin(0), 0*sin(theta6);
      0, sin(0), cos(0), 82.5;
      0, 0, 0, 1;];
t67 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, L; 0, 0, 0, 1;];

t02 = t01*t12;
t03 = t01*t12*t23;
t04 = t01*t12*t23*t34;
t05 = t01*t12*t23*t34*t45;
t06 = t01*t12*t23*t34*t45*t56;
t07 = t01*t12*t23*t34*t45*t56*t67;

z0 = [0; 0; 1;];
o0 = [0; 0; 0;];
z1 = t01(1:3,3);
o1 = t01(1:3,4);
z2 = t02(1:3,3);
o2 = t02(1:3,4);
z3 = t03(1:3,3);
o3 = t03(1:3,4);
z4 = t04(1:3,3);
o4 = t04(1:3,4);
z5 = t05(1:3,3);
o5 = t05(1:3,4);
z6 = t06(1:3,3);
o6 = t06(1:3,4);
z7 = t07(1:3,3);
o7 = t07(1:3,4);

J = [cross(z0,(o7-o0)) cross(z1,(o7-o1)) cross(z2,(o7-o2)) cross(z3,(o7-o3)) cross(z4,(o7-o4)) cross(z5,(o7-o5)) cross(z6,(o7-o6));
    z0 z1 z2 z3 z4 z5 z6];
end
