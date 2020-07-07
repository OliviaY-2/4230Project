%{
Check if VM is connected to wired network (top right hand corner) 
In VM command window find IP addr (double check) $ ifconfig
To check your pc can connect, use local command window $ ping 192.***.**.** 

In VM command window, launch Gazebo environemnt with arm
cd ~/simulation_ws/
catkin_make
source devel/setup.bash
export SVGA_VGPU10=0
export ROS_IP=<your_vms_ip_address> % EXTRA COMMAND IF MATLAB CAN'T CONNECT
roslaunch ur5_t2_4230 ur5_world.launch

In new terminal, open rviz to see camera outputs
rviz

to add object
in gazebo, click the shapes in the bar at the top of the panel

%} 
%following script retrieves a single RGB-D image from the kinect camera in Gazebo

clear all;
close all;
clc;

ipaddress = '192.168.1.120';
robotType = 'Gazebo';
rosshutdown;
rosinit(ipaddress);
blockposes = rossubscriber('/gazebo/link_states');
pause(2);
posdata = receive(blockposes, 10);
imsub = rossubscriber('/camera/color/image_raw');
pcsub = rossubscriber('/camera/depth/points');

figure(1);
testIm = readImage(imsub.LatestMessage);
imshow(testIm);

figure(2);
scatter3(pcsub.LatestMessage);

%debug info about the ros topic eg subscribers, publishers etc
rostopic list;

%{ 
Commands that can be used with data type 'pointcloud2'
% Get RGB info and xyz-coordinates from the 
% point cloud using readXYZ and readRGB.
xyz = readXYZ(ptcloud2);
rgb = readRGB(ptcloud2);
% plot point cloud
scatter3(ptcloud2)
%}

