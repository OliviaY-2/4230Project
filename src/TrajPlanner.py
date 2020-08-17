

#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from ur5_t2_4230.msg import Tracker
from std_msgs.msg import Bool
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

tracker = Tracker()
bin_x = -0.5
bin_y = 0.11
travel_height = 0.4
pick_height = 0.05


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    self.cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)
    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print ("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print ("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print ("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    print (robot.get_current_state())
    print (" ")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -pi/2
    joint_goal[1] = -pi/2
    joint_goal[2] = pi/2
    joint_goal[3] = -pi/2
    joint_goal[4] = -pi/2
    joint_goal[5] = 0
    #joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    #bin position
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.48
    pose_goal.position.y = 0.109217
    pose_goal.position.z = 0.432

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale = 1):
  # Copy class variables to local variables to make the web tutorials more clear.
  # In practice, you should use the class variables directly unless you have a good
  # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []
    # start_pose = self.get_current_pose(eef_link).pose
    # print(start_pose.position)
    wpose = move_group.get_current_pose().pose
    #waypoints.append(start_pose)
   
    #wpose = deepcopy(start_pose)
   # wpose = move_group.get_current_pose().pose

    # # Set the next waypoint to location of the pick object
    # wpose.position.x = data.x
    # wpose.position.y = data.y
    # wpose.position.z = travel_height
    # waypoints.append(deepcopy(wpose))

    # lower to pick the object

    # wpose.position.x = data.x
    # wpose.position.y = data.y
    
    wpose.position.z = pick_height
    tracker.flag2 = 1
    waypoints.append(copy.deepcopy(wpose))
    self.cxy_pub.publish(tracker)
      

    # raise
    # wpose.position.x = data.x
    # wpose.position.y = data.y..............
    wpose.position.z = travel_height
    waypoints.append(copy.deepcopy(wpose))
    
    #move to the bin
    wpose.position.x = bin_x
    wpose.position.y = bin_y
    wpose.position.z = travel_height
    waypoints.append(copy.deepcopy(wpose))

    # place in the bin
    wpose.position.x = bin_x
    wpose.position.y = bin_y
    wpose.position.z = pick_height
    tracker.flag2 = 0
    waypoints.append(copy.deepcopy(wpose))
    self.cxy_pub.publish(tracker)


    #retract from the bin
    wpose.position.x = bin_x
    wpose.position.y = bin_y
    wpose.position.z = travel_height
    waypoints.append(copy.deepcopy(wpose))


    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

## END_SUB_TUTORIAL
def main():
  try:
    print ("")
    print ("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print ("============ Press `Enter` to execute a movement using a joint state goal ...")
    raw_input()
    tutorial.go_to_joint_state()

    # print "============ Press `Enter` to execute a movement using a pose goal ..."
    # raw_input()
    # tutorial.go_to_pose_goal()

    print ("============ Press `Enter` to plan and display a Cartesian path ...")
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    print ("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
    raw_input()
    tutorial.display_trajectory(cartesian_plan)

    print ("============ Press `Enter` to execute a saved path ...")
    raw_input()
    tutorial.execute_plan(cartesian_plan)

    # print "============ Press `Enter` to add a box to the planning scene ..."
    # raw_input()
    # tutorial.add_box()

    # print "============ Press `Enter` to attach a Box to the Panda robot ..."
    # raw_input()
    # tutorial.attach_box()

    # print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
    # raw_input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    # tutorial.execute_plan(cartesian_plan)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return  

if __name__ == "__main__":
  main()
