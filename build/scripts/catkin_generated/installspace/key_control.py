#!/usr/bin/env python3

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
    # msg.axes[x]:	Left stick:  	L/R 0 (L= -1, R= +1)
    #    		Left stick:  	U/D 1 (U = +1, D = -1)
    #    		Left trigger  	    2 (Out = 1, In = -1)
    #    	  	Right stick: 	L/R 3 (L= -1, R= +1)
    #    	  	Right stick: 	U/D 4 (U = +1, D = -1)
    #    	  	Right trigger:	    5 (Out = 1, In = 0)
    #    	  	D-pad:	     	L/R 6 (L = -1, R = 1)
    #    	  	D-pad:	     	U/D 7 (U = -1, R = 1)
    #
    # msg.buttons[x]: 	X  0
    # 			C  1
    #			T  2
    #			S  3
    #			LB 4
    #			RB 5
    #			Sh 8
    #			Op 9
    #			PS 10
    #			L3 11
    #			R3 12

    # Convert msg into 1x8 axes list & 1x11 button list
    self.ax = [msg.axes[0], msg.axes[1], msg.axes[2], msg.axes[3], msg.axes[4], msg.axes[5], msg.axes[6], msg.axes[7]]
    self.but = [msg.buttons[0], msg.buttons[1], msg.buttons[2], msg.buttons[3], msg.buttons[4], msg.buttons[5], msg.buttons[8], msg.buttons[9], msg.buttons[10], msg.buttons[11], msg.buttons[12]]    



  def joy_pose_goal(self):
    control_mode = "joy" #"key"/"joy" Toggle for keyboard/joystick operation

    o_tolerance = 0.1     # Orientation tolerance (rads)
    p_tolerance = 0.001   # Pose tolerance (metres)
    j_tolerance = 0.1     # Joint tolerance (rads)
    
    step_size = 0.01      # Movement step size (metres)
    input_rate = 30       # Loop rate (Hz)

    move_group = self.move_group
    robot = self.robot
    pose_goal = geometry_msgs.msg.Pose()
    
    # Set tolerances
    move_group.set_goal_orientation_tolerance(o_tolerance)
    move_group.set_goal_position_tolerance(p_tolerance)
    move_group.set_goal_joint_tolerance(j_tolerance)

    # Get start position
    [current_r, current_p, current_y] = move_group.get_current_rpy()
    [o_x, o_y, o_z, o_w] = EulerToQuaternion(current_r, current_p, current_y)
    p_x = move_group.get_current_pose().pose.position.x
    p_y = move_group.get_current_pose().pose.position.y
    p_z = move_group.get_current_pose().pose.position.z
    
    if control_mode == "joy":
      print('######### starting joystick control')
      print('######### X-axis -/+: L/R L Stick, Y-axis -/+: U/D L stick, Z-axis -/+: U/D D-Pad')

      # Initialise axis & buttons arrays
      self.ax = [] * 8
      self.but = [] * 11
      
      while not rospy.is_shutdown():
          # Set while loop rate
          rate = rospy.Rate(input_rate)
          
          # read & log joystick input
          #start_time = time.time()   # for debugging
          # JOYSTICK CONTROL
          rospy.Subscriber("joy", Joy, self.callback, queue_size=1)
          #print('---Time to callback Joy: %s seconds ---' % (time.time() - start_time)) # for debugging
    
          # If a command has been received plan a motion for this group to a desired pose for the end-effector:
          if self.ax or self.but:  # if joystick input received
              # Get updated start position
              [current_r, current_p, current_y] = move_group.get_current_rpy()
              [o_x, o_y, o_z, o_w] = EulerToQuaternion(current_r, current_p, current_y)
              p_x = move_group.get_current_pose().pose.position.x
              p_y = move_group.get_current_pose().pose.position.y
              p_z = move_group.get_current_pose().pose.position.z

              #start_time = time.time() # FOR DEBUGGING

              # Update Goal position (Orientation TO DO) & set pose target
              o_x = o_x
              o_y = o_y
              o_z = o_z
              o_w = o_w
              p_x = p_x + step_size * self.ax[1] # left/right left joystick
              p_y = p_y + step_size * self.ax[0] # up/down left joystick
              p_z = p_z + step_size * self.ax[7] # up/down d-pad
              
              pose_goal.orientation.x = o_x
              pose_goal.orientation.y = o_y
              pose_goal.orientation.z = o_z
              pose_goal.orientation.w = o_w
              pose_goal.position.x = p_x
              pose_goal.position.y = p_y
              pose_goal.position.z = p_z
              
              move_group.set_pose_target(pose_goal)		# Set goal
              #print('---Time to set goal: %s seconds ---' % (time.time() - start_time)) # FOR DEBUGGING

              #start_time = time.time() # FOR DEBUGGING
              plan = move_group.go()				# Call the planner to compute the plan and execute it.
              move_group.stop()					# Ensures that there is no residual movement
              move_group.clear_pose_targets()			# Clear targets after planning with poses.
              #print('---Time to compute plan and execute: %s seconds ---' % (time.time() - start_time)) # FOR DEBUGGING
              
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
          p_y = p_y + step_size   # move forwards
        elif ch == 's':
          p_y = p_y - step_size   # move backwards
        elif ch == 'd':
          p_x = p_x + step_size   # move right
        elif ch == 'a':
          p_x = p_x - step_size   # move left
        elif ch == 'e':
          p_z = p_z + step_size   # move up
        elif ch == 'q':
          p_z = p_z + step_size   # move down

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


