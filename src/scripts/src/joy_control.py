#!/usr/bin/env python

# Python 2/3 compatibility imports
#from __future__ import print_function
#from six.moves import input

import sys
import copy
import rospy
import roscpp
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import Joy
import time
import numpy as np
#import getch


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


def EulerToQuaternion(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]      


class MoveGroupPythonInterface(object):
  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()

    joint_state_topic = ['joint_states:=/iiwa/joint_states']      # remap joint_states to /iiwa/joint_states topic
    robot_description = "/iiwa/robot_description"
    group_name = "manipulator"
    ns = "iiwa"
    
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(joint_state_topic)         # initialize remapped topic
    rospy.init_node('move_group_python_interface', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander(robot_description)

    ## Instantiate a `PlanningSceneInterface`_ object.  For getting, setting, and updating the robot's internal understanding of the surrounding world:
    #scene = moveit_commander.PlanningSceneInterface()
    #print('------planning scene interface object instantiated')
    
    ## Instantiate a `MoveGroupCommander`_ object - Interface to a planning group (group of joints).  
    move_group = moveit_commander.MoveGroupCommander(group_name, robot_description, ns)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Getting Basic Information
    planning_frame = move_group.get_planning_frame() 	# get reference frame
    eef_link = move_group.get_end_effector_link() 	# get end effector link group name
    group_names = robot.get_group_names() 		# get group names
    current_state = robot.get_current_state() 		# get current state

    # Misc variables
    self.box_name = ''
    self.robot = robot
    #self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    print('------set variables')


  def callback(self, msg):
    # Convert msg into 1x8 axes list & 1x11 button list

    self.ax = [msg.axes[0], msg.axes[1], msg.axes[2], msg.axes[3], msg.axes[4], msg.axes[5], msg.axes[6], msg.axes[7]]
    # -1/+1:  [ Left L/R     Left D/U     L2 In/Out    Right L/R    Right U/D    R2 In/out    D-pad L/R    D-pad D/U ]
    
    self.but = [msg.buttons[0], msg.buttons[1], msg.buttons[2], msg.buttons[3], msg.buttons[4], msg.buttons[5], msg.buttons[8], msg.buttons[9], msg.buttons[10], msg.buttons[11], msg.buttons[12]]    
    # 0/1:     [ X                Circle          Triangle        Square          L1              R1              Share           Options         PS Button       L3                R3           ]


  def joy_pose_goal(self):
    control_mode = "joy"  # "key"/"joy" Toggle for keyboard/joystick operation

    o_tolerance = pi/1800 # Orientation tolerance (rads)
    p_tolerance = 0.001   # Pose tolerance (metres)
    j_tolerance = 0.1     # Joint tolerance (rads)
    
    p_step_size = 0.01    # Position step size (metres)
    o_step_size = pi/180  # Orientation step size (rads)
    input_rate = 30       # Loop rate (Hz)

    move_group = self.move_group
    robot = self.robot
    pose_goal = geometry_msgs.msg.Pose()
    
    # Set tolerances
    move_group.set_goal_orientation_tolerance(o_tolerance)
    move_group.set_goal_position_tolerance(p_tolerance)
    move_group.set_goal_joint_tolerance(j_tolerance)
    
    if control_mode == "joy":
      print('######### starting joystick control')
      print('######### X-axis -/+: L/R L Stick, Y-axis -/+: U/D L stick, Z-axis -/+: U/D D-Pad')
      print('######### Roll -/+: L/R R Stick, Pitch -/+: U/D R stick, Yaw -/+: L1/R1')

      # Initialise axis & buttons arrays
      self.ax = [] * 8
      self.but = [] * 11
      
      while not rospy.is_shutdown():
          # Set while loop rate
          rate = rospy.Rate(input_rate)
          
          # read & log joystick input
          rospy.Subscriber("joy", Joy, self.callback, queue_size=1)
    
          # If a command has been received plan a motion for this group to a desired pose for the end-effector:
          if self.ax or self.but:                                                                   # If joystick input received
              # Calculate new goal pose based on current pose
              # Position
              p_x = move_group.get_current_pose().pose.position.x + p_step_size * self.ax[1]        # Left/right left joystick
              p_y = move_group.get_current_pose().pose.position.y + p_step_size * self.ax[0]        # Up/down left joystick
              p_z = move_group.get_current_pose().pose.position.z + p_step_size * self.ax[7]        # Up/down d-pad

              # Orientation
              [current_r, current_p, current_y] = move_group.get_current_rpy()
              new_r = current_r + o_step_size * self.ax[3]                                      # Left/right right joystick
              new_p = current_p + o_step_size * self.ax[4]                                      # Up/down right joystick
              new_y = current_y - o_step_size * self.but[4] + o_step_size * self.but[5]         # L1/R1
              [o_x, o_y, o_z, o_w] = EulerToQuaternion(new_r, new_p, new_y)                     # Convert to quarternions
            
              # Assign goal pose values
              pose_goal.orientation.x = o_x
              pose_goal.orientation.y = o_y
              pose_goal.orientation.z = o_z
              pose_goal.orientation.w = o_w
              pose_goal.position.x = p_x
              pose_goal.position.y = p_y
              pose_goal.position.z = p_z
              
              # Set goal
              move_group.set_pose_target(pose_goal)		

              # Execute plan
              plan = move_group.go()				# Call the planner to compute the plan and execute it.
              move_group.stop()					# Ensures that there is no residual movement
              move_group.clear_pose_targets()			# Clear targets after planning with poses.
              
          rate.sleep()    
    
    if control_mode == "key":
      print('######### starting keyboard control')
      print('######### X-axis -/+: a/d, Y-axis -/+: w/s, Z-axis -/+: q/e')
      
      while not rospy.is_shutdown():
        # Set while loop rate
        rate = rospy.Rate(input_rate)

        # Get updated start position
        [current_r, current_p, current_y] = move_group.get_current_rpy()
        [o_x, o_y, o_z, o_w] = EulerToQuaternion(current_r, current_p, current_y)
        p_x = move_group.get_current_pose().pose.position.x
        p_y = move_group.get_current_pose().pose.position.y
        p_z = move_group.get_current_pose().pose.position.z

        # Get keyboard input & update goal  
        #ch = getch.getch()
        ch = ''
        if ch == 'w':
          p_y = p_y + p_step_size   # move forwards
        elif ch == 's':
          p_y = p_y - p_step_size   # move backwards
        elif ch == 'd':
          p_x = p_x + p_step_size   # move right
        elif ch == 'a':
          p_x = p_x - p_step_size   # move left
        elif ch == 'e':
          p_z = p_z + p_step_size   # move up
        elif ch == 'q':
          p_z = p_z + p_step_size   # move down

        pose_goal.orientation.x = o_x
        pose_goal.orientation.y = o_y
        pose_goal.orientation.z = o_z
        pose_goal.orientation.w = o_w
        pose_goal.position.x = p_x
        pose_goal.position.y = p_y
        pose_goal.position.z = p_z
        
        move_group.set_pose_target(pose_goal)		# Set goal
        plan = move_group.go()				# Call the planner to compute the plan and execute it.
        move_group.stop()				# Ensures that there is no residual movement
        move_group.clear_pose_targets()			# Clear targets after planning with poses.

        ch = None                                       # Reset ch
        
        rate.sleep()                                    # Sleep to match given rate
      
    # For testing:
    #current_pose = self.move_group.get_current_pose().pose
    #return all_close(pose_goal, current_pose, 0.01)



def main():
  try:  
    MoveGroupPythonInterface().joy_pose_goal()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()


