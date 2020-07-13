%%
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
%}
%% INSTRUCTIONS
%{ 
Run 'ROS_Connection.m' in sections. Have Gazebo open and make sure the IP
address matches the one you are using.

1. Run and advance the 'Set up connection to gazebo' section. You only 
need to run this once per session.
2. To collect training data, 'Run and Advance' the 'Receive new data' 
section, wait for it to finish.
3. Then to save the training data, update 'filename' to whatever you want to 
call this particular data set. This variable is found in the 'Save RGBD Data' section.
4. 'Run and advance' the 'Save RGBD Data' section.

Repeat steps 2-4 for each new data set. Remember to change the camera angle 
or object position in Gazebo so that each data set is unique and useful.
%}
%% Run this code using 'Run Section' or 'Run an Advance' to reduce time (no need to re-establish connection to gazebo every time)
%% Set up connection to gazebo
tic
clear all;
close all;
clc;

ipaddress = '192.168.1.116';
robotType = 'Gazebo';
rosshutdown;
rosinit(ipaddress);
blockposes = rossubscriber('/gazebo/link_states');
pause(2);

toc
disp('Finished initialisation');
%% Receive new data
disp("Getting new image..");
tic
posdata = receive(blockposes, 10);
imsub = rossubscriber('/camera/color/image_raw');
pcsub = rossubscriber('/camera/depth/points');
pause(2);
testIm = readImage(imsub.LatestMessage);
%figure(1);
%imshow(testIm);

%debug info about the ros topic eg subscribers, publishers etc
%rostopic list;

% plot the depth data with rgb
depthxyz = readXYZ(pcsub.LatestMessage);
depthrgb = readRGB(pcsub.LatestMessage);
%figure(2);
%pcshow(depthxyz,depthrgb);% remove rgb if you don't want colours

disp("Finished getting new image.");

% Save RGBD Data - Remember to change 'filename' for each new data set
% UNCOMMENT THIS SECTION IF YOU WANT TO SAVE RGBD DATA
% xyz = xyz coords of each point in the point cloud
% rgb = the rgb values of each point in the point cloud
% image = the rgb image
disp("Saving new image..");

% save the xyz and rgb data into a struct, then export
filename = '05_Cubes_and_ Cylinders.mat' % Change this to whatever you want to call this data set
full_filename = fullfile('.\RGBD_Data\',filename);
s1.xyz = depthxyz; % matlab automatically recognises s1 as a struct
s1.rgb = depthrgb;
s1.image = testIm;
save(full_filename,'-struct','s1'); % input order: filename, datattype 'struct', struct

%{
% try reloading our saved file to check that it was successful
figure(3);
test = load(full_filename);
pcshow(test.xyz,test.rgb);
figure(4); 
imshow(test.image);
%}
toc
disp('Saved!');
%%


