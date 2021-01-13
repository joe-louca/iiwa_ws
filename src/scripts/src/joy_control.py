#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import Joy
import time


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

      

class MoveGroupPythonInterface(object):
  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()

    robot_description = "/iiwa/robot_description"
    group_name = "manipulator"
    ns = "iiwa"
    
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    print('------moveit commander node initialized')
    rospy.init_node('move_group_python_interface', anonymous=True)
    print('------move_group_python_interface rospy node initialized')


    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander(robot_description)
    # CANNOT FIND ROBOT SEMANTIC DESCRIPTION HERE. REMAPPED from /robot_description_semantic TO /iiwa/robot_description_semantic
    print('------robot commander object instantiated')

    
    ## Instantiate a `PlanningSceneInterface`_ object.  For getting, setting, and updating the robot's internal understanding of the surrounding world:
    #scene = moveit_commander.PlanningSceneInterface()
    #print('------planning scene interface object instantiated')
    
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface to a planning group (group of joints).  
    ## This interface can be used to plan and execute motions:
    move_group = moveit_commander.MoveGroupCommander(group_name, robot_description, ns)
    print('------movegroupcommander object instantiated')

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    print('------display trajectory ros publisher created')

    ## Getting Basic Information
    planning_frame = move_group.get_planning_frame() 	# get reference frame
    eef_link = move_group.get_end_effector_link() 		# get end effector link group name
    group_names = robot.get_group_names() 			# get group names
    current_state = robot.get_current_state() 		# get current state
    print('------got basic information')
    print('planning frame: ' + planning_frame)
    print('eef_link: ' + eef_link)
    print('group names:')
    print(group_names)
    print('current_state:')
    print(current_state)

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

    #rospy.loginfo("axes: {}".format(self.ax))
    #rospy.loginfo("buttons: {}".format(self.but))


  def joy_pose_goal(self):
    step_size = 0.02
    input_rate = rospy.Rate(30) # Hz
    self.ax = [] * 8
    self.but = [] * 11
    
    move_group = self.move_group
    pose_goal = geometry_msgs.msg.Pose()
    # set start position
    o_x = 0.0
    o_y = 0.0
    o_z = 0.0
    o_w = 1.0
    p_x = 0.0
    p_y = 0.0
    p_z = 1.0 
        
    pose_goal.orientation.w = o_w
    pose_goal.position.x = p_x
    pose_goal.position.y = p_y
    pose_goal.position.z = p_z
    print('moving to start position')    
    move_group.set_pose_target(pose_goal)		# Set goal

    plan = move_group.go()				# Call the planner to compute the plan and execute it.
    move_group.stop()					# Ensures that there is no residual movement
    move_group.clear_pose_targets()			# Clear targets after planning with poses.

    print('######### starting joystick control')
    
    while not rospy.is_shutdown():
        # read & log joystick input
        start_time = time.time()
        rospy.Subscriber("joy", Joy, self.callback, queue_size=1)
        print('---Time to callback Joy: %s seconds ---' % (time.time() - start_time))
        
        # If a command has been received plan a motion for this group to a desired pose for the end-effector:
        if self.ax or self.but:  # if joystick input received
            start_time = time.time()
            o_w = 1.0
            p_x = p_x + step_size * self.ax[1] # left/right left joystick
            p_y = p_y + step_size * self.ax[0] # up/down left joystick
            p_z = p_z + step_size * self.ax[7] # up/down d-pad
            
            pose_goal.orientation.w = o_w
            pose_goal.position.x = p_x
            pose_goal.position.y = p_y
            pose_goal.position.z = p_z
            #print(pose_goal)
            
            move_group.set_pose_target(pose_goal)		# Set goal
            print('---Time to set goal: %s seconds ---' % (time.time() - start_time))

            start_time = time.time()
            plan = move_group.go()				# Call the planner to compute the plan and execute it.
            move_group.stop()					# Ensures that there is no residual movement
            move_group.clear_pose_targets()			# Clear targets after planning with poses.
            print('---Time to compute plan and execute: %s seconds ---' % (time.time() - start_time))
            
    input_rate.sleep()    
    
    
    # For testing:
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)



def main():
  try:  
    MoveGroupPythonInterface().joy_pose_goal()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()


