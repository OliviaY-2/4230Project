This contains the robot arm with grippers and no kinect camera since we're now using a static camera thats floating
above the table. This is to ensure that there wont be coexisiting cameras in the simulation environment.

Only copy and paste these files to src/universal_robot/ur_description/urdf

common.gazebo.xacro;
ur5.urdf.xacro;
ur5_joint_limited_robot.urdf.xacro;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Recently added the code for activating the grippers

-Simply copy and paste the src folder into ur5_t2_4230
-Make sure in CMakeLists.txt u have the same settings as I do. 
  e.g. 
  ## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   blocks_poses.msg
   Tracker.msg
 )
 
 ## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )
