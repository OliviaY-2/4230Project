This contains the robot arm with grippers and no kinect camera since we're now using a static camera thats floating
above the table. This is to ensure that there wont be coexisiting cameras in the simulation environment.

Only copy and paste these files to src/universal_robot/ur_description/urdf

common.gazebo.xacro;
ur5.urdf.xacro;
ur5_joint_limited_robot.urdf.xacro;
