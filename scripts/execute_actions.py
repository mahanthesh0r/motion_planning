#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_move_it_trajectories.py __ns:=my_gen3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from math import sqrt, inf, degrees, radians
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_slerp
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive


class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    self.object_location = None
    self.msg_received = False

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        self.gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True

    
    #Subscribers
    rospy.Subscriber('/object_location', String, self.object_location_callback)


  
  def object_location_callback(self, msg):
    self.object_location = self.parse_location(msg.data)
    self.msg_received = True

  
  def parse_location(self, msg):
    location = {}
    try:
        # Split the string into individual coordinate strings
        coords = msg.split()
        
        for i in range(0, len(coords), 2):
            key = coords[i].rstrip(':')  # Remove trailing colon
            value = coords[i + 1]
            try:
                # Convert value to float after stripping any extra whitespace
                location[key] = float(value)
            except ValueError:
                rospy.logwarn("Skipping coordinate with invalid float value: %s", value)
    except Exception as e:
        rospy.logerr("Error parsing location: %s", e)
    return location

  def get_object_location(self):
    rospy.loginfo("Waiting for object location...")
    while not self.msg_received:
        rospy.sleep(0.1)
    return self.object_location
  


  def sip(self):
    rospy.loginfo("Sipping...")
    print("Focusing camera...")
    rospy.sleep(1.0)
    #self.focus_camera()
    print("Done.")
    rospy.sleep(2.0)

    location = self.get_object_location()
    x = location.get('x', 0)
    y = location.get('y', 0)
    z = location.get('z', 0)
    roll = location.get('roll', 0)
    pitch = location.get('pitch', 0)
    yaw = location.get('yaw', 0)
    print("Object Location before: ", x, y, z, roll, pitch, yaw)

    if(z < 0.165):
        z = 0.165
    
    # Refine the object oreintation
    roll = -roll #for some reason the roll sign has to be flipped
    yaw = (yaw + 180) % 360

    print("Object Location after: ", x, y, z, roll, pitch, yaw)

    # convert to quaternions
    roll = radians(roll)
    pitch = radians(pitch)
    yaw = radians(yaw)
    q = quaternion_from_euler(roll, pitch, yaw)

    # Construct pose
    P = self.get_cartesian_pose()
    P.position.x = x
    P.position.y = y
    P.position.z = z
    P.orientation.x = q[0]
    P.orientation.y = q[1]
    P.orientation.z = q[2]
    P.orientation.w = q[3]

    # Set Constraints
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.position.x = 0.5
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 2
    box_pose.pose.orientation.w = 1.0

    box_size = [0.5, 0.5, 0.3] # x, y, z in meters

    box_id = "boundary_box"
    self.scene.add_box(box_id, box_pose, size=box_size)

    # Set the constraints
    box_constraint = moveit_msgs.msg.Constraints()
    position_constraint = moveit_msgs.msg.PositionConstraint()
    position_constraint.header.frame_id = "base_link"
    position_constraint.link_name = "tool_frame"

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = box_size

    position_constraint.constraint_region.primitives.append(box)
    position_constraint.constraint_region.primitive_poses.append(box_pose.pose)
    position_constraint.weight = 1.0

    box_constraint.position_constraints.append(position_constraint)


    joint_constraints = moveit_msgs.msg.JointConstraint()
    joint_constraints.joint_name = "joint_5"
    joint_constraints.position = -2.070
    joint_constraints.tolerance_above = 0.2
    joint_constraints.tolerance_below = 0.8
    joint_constraints.weight = 1.0

    constraints = moveit_msgs.msg.Constraints()
    constraints.joint_constraints.append(joint_constraints)





    success = self.move('pose', P, tolerance=0.01, vel=0.5, accel=0.5, attempts=10, time=100.0, constraints=None)
    if success:
        print("Done.")
          # Close the gripper
        success = self.move('gripper', 0.8)
        
        
    else:
        print("Move to cup failed.")
        #self.reset()

      
  def prepare_object_location(self):
      location = self.get_object_location()
      x = location.get('x', 0)
      y = location.get('y', 0)
      z = location.get('z', 0)
      roll = location.get('roll', 0)
      pitch = location.get('pitch', 0)
      yaw = location.get('yaw', 0)
      print("Object Location before: ", x, y, z, roll, pitch, yaw)

      # if(z < 0.170):
      #   z = 0.170

      roll = -roll #for some reason the roll sign has to be flipped

      print("Object Location after: ", x, y, z, roll, pitch, yaw)

      # convert to quaternions
      roll = radians(roll)
      pitch = radians(pitch)
      yaw = radians(yaw)
      q = quaternion_from_euler(roll, pitch, yaw)

      # Construct pose
      P = self.get_cartesian_pose()
      P.position.x = x
      P.position.y = y
      P.position.z = z
      P.orientation.x = q[0]
      P.orientation.y = q[1]
      P.orientation.z = q[2]
      P.orientation.w = q[3]

      return P
  
  def prepare_grasp(self):
    rospy.loginfo("Preparing to grasp...")
    grasp_scan = [0.000, 0.000, -1.571, 0.000, -1.571, 1.571]
    # Move to grasp scan location
    rospy.loginfo("Moving to grasp scan location...")
    success = self.move('joint', grasp_scan, tolerance=0.01, vel=1.0, accel=1.0, attempts=20, time=20.0, constraints=None)
    if success:
      success = self.move('gripper', 0.60)
    else:
      print("Move to grasp scan failed.")
      return

    



  def grasp(self):
      rospy.loginfo("Grasping...")
      print("Focusing camera...")
      rospy.sleep(1.0)
      #self.focus_camera()
      print("Done.")
      rospy.sleep(2.0)

      feed_grasp = [0.281678688, 0.2402271182, -1.8603813563, 0.0112050138, -1.0966950888, 1.5619475075]
      # Move to Grasp Scan
      self.prepare_grasp()

      rospy.sleep(6.0)
      

      # Get object location
      P = self.prepare_object_location()

      # Set Constraints
      box_pose = PoseStamped()
      box_pose.header.frame_id = "base_link"
      box_pose.pose.position.x = 0.5
      box_pose.pose.position.y = 0.0
      box_pose.pose.position.z = 2
      box_pose.pose.orientation.w = 1.0

      box_size = [0.5, 0.5, 0.3] # x, y, z in meters

      box_id = "boundary_box"
      self.scene.add_box(box_id, box_pose, size=box_size)

      # Set the constraints
      box_constraint = moveit_msgs.msg.Constraints()
      position_constraint = moveit_msgs.msg.PositionConstraint()
      position_constraint.header.frame_id = "base_link"
      position_constraint.link_name = "tool_frame"

      box = SolidPrimitive()
      box.type = SolidPrimitive.BOX
      box.dimensions = box_size

      position_constraint.constraint_region.primitives.append(box)
      position_constraint.constraint_region.primitive_poses.append(box_pose.pose)
      position_constraint.weight = 1.0

      box_constraint.position_constraints.append(position_constraint)


      joint_constraints = moveit_msgs.msg.JointConstraint()
      joint_constraints.joint_name = "joint_5"
      joint_constraints.position = -2.070
      joint_constraints.tolerance_above = 0.2
      joint_constraints.tolerance_below = 0.8
      joint_constraints.weight = 1.0

      constraints = moveit_msgs.msg.Constraints()
      constraints.joint_constraints.append(joint_constraints)





      success = self.move('pose', P, tolerance=0.01, vel=0.5, accel=0.5, attempts=10, time=100.0, constraints=None)
      if success:
        self.handle_grasp()
      else:
          print("Move to cup failed.")
          #self.reset()

  def handle_grasp(self):
    print("Done.")
    #Close the gripper
    success = self.move('gripper', 0.9)
    if success:
        grasp_success = input("Did the robot grasp the object? (y/n): ")
        if grasp_success == 'y':
          print("Grasping successful.")
          feed_grasp_joints = [1.4378760925762786, -0.047248738795749645, -1.1988789563464968, 0.8093016986565907, -1.9336991632694867, 0.23040917408357622]
          success2 = self.move('joint',feed_grasp_joints, tolerance=0.01, vel=1.0, accel=1.0, attempts=20, time=10.0, constraints=None)
          rospy.sleep(2.0)
          success = self.move('gripper', 0.65)
          home_joints = [0.3898367417254534, -1.3306542976124893, -1.8414975968537446, 0.10480068192728456, -1.7593289572126576, 1.5593387419951033]
          success3 = self.move('joint', home_joints, tolerance=0.01, vel=1.0, accel=1.0, attempts=20, time=10.0, constraints=None)
          success_g4 = self.move('gripper', 0)
        elif grasp_success == 'n':
          self.retry_grasp()

  def retry_grasp(self):
    # Construct Retry Pose
    rospy.loginfo("Retrying grasp...")
    P = self.get_cartesian_pose()

    P.position.z = 0.3
    euler_angles = euler_from_quaternion([P.orientation.x, P.orientation.y, P.orientation.z, P.orientation.w])
    roll = degrees(euler_angles[0])
    pitch = degrees(euler_angles[1])
    yaw = 90

    q = quaternion_from_euler(radians(roll), radians(pitch), radians(yaw))

    P.orientation.x = q[0]
    P.orientation.y = q[1]
    P.orientation.z = q[2]
    P.orientation.w = q[3]

    #Move to Retry Pose
    
    success = self.move('pose', P, tolerance=0.01, vel=0.5, accel=0.5, attempts=10, time=10.0, constraints=None)
    rospy.sleep(2.0)
    if success:
      success_gripper = self.move('gripper', 0.65)
    
    # Get object location
    rospy.sleep(4.0)
    P_new = self.prepare_object_location()
    
    if P_new is not None:
      success = self.move('pose', P_new, tolerance=0.01, vel=0.5, accel=0.5, attempts=10, time=10.0, constraints=None)
      self.handle_grasp()

        


  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(trajectory_message, wait=True)

  def reach_joint_angles(self, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = pi/2
      joint_positions[1] = 0
      joint_positions[2] = pi/4
      joint_positions[3] = -pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
      joint_positions[6] = 0.2
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = 0
      joint_positions[1] = 0
      joint_positions[2] = pi/2
      joint_positions[3] = pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 
    

  def move(self, goal_type, goal, tolerance=0.01, vel=0.5, accel=0.5, attempts=10, time=5.0, constraints=None):
    arm_group = self.arm_group

    # Set Parameters
    arm_group.set_planner_id("STOMP")
    arm_group.set_max_velocity_scaling_factor(vel)
    arm_group.set_max_acceleration_scaling_factor(accel)
    arm_group.set_num_planning_attempts(attempts)
    arm_group.set_planning_time(time)
  
    

    if goal_type == 'pose':
      arm_group.clear_pose_targets()
      arm_group.set_goal_position_tolerance(tolerance)


      if constraints is not None:
        arm_group.set_path_constraints(constraints)

      
      arm_group.set_pose_target(goal)

      #plan and execute
      (success, plan, planning_time, error_code) = arm_group.plan()
      if success:
        print("Planning was successful")
        print(f"Panning time: {planning_time}")
        print("Executing the plan")
        joint_positions = plan.joint_trajectory.points[-1].positions

        print("Last Planning Angles: ", [degrees(joint_positions[i]) for i in range(len(joint_positions))])
        print("Planning size: ", len(plan.joint_trajectory.points))
  
        success = arm_group.execute(plan, wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
      else:
        print("Planning failed")
      
    elif goal_type == 'joint':
      # Get the current joint positions
      joint_positions = arm_group.get_current_joint_values()

      # Set the goal joint tolerance
      self.arm_group.set_goal_joint_tolerance(tolerance)

      # Set the joint target configuration
      joint_positions[0] = goal[0]
      joint_positions[1] = goal[1]
      joint_positions[2] = goal[2]
      joint_positions[3] = goal[3]
      joint_positions[4] = goal[4]
      joint_positions[5] = goal[5]
      arm_group.set_joint_value_target(joint_positions)

      # Plan & Execute
      (success, plan, planning_time, error_code) = arm_group.plan()
      if success:
          print("Planning Successful.")
          print(f"Planning time: {planning_time}")
          print("Executing Plan...")
          success = arm_group.execute(plan, wait=True)
          arm_group.stop()

    elif goal_type == 'path':
      # Clear old pose targets
      arm_group.clear_pose_targets()

      # Clear max cartesian speed
      arm_group.clear_max_cartesian_link_speed()

      # Set the tolerance
      arm_group.set_goal_position_tolerance(tolerance)

      # Set the trajectory constraint if one is specified
      if constraints is not None:
          arm_group.set_path_constraints(constraints)

      eef_step = 0.01
      jump_threshold = 0.0
      (plan, fraction) = arm_group.compute_cartesian_path(goal, eef_step, jump_threshold)
      success = arm_group.execute(plan, wait=True)
      arm_group.stop()

    elif goal_type == 'gripper':
      # We only have to move this joint because all others are mimic!
      gripper_joint = self.robot.get_joint(self.gripper_joint_name)
      gripper_max_absolute_pos = gripper_joint.max_bound()
      gripper_min_absolute_pos = gripper_joint.min_bound()
      success = gripper_joint.move(goal * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
    
    else:
      rospy.ERROR("Invalid Goal Type.")

    return success


def main():
  example = ExampleMoveItTrajectories()

  example.grasp()
  #example.sip()


  # # For testing purposes
  # success = example.is_init_success
  # try:
  #     rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  # except:
  #     pass

  # if success:
  #   rospy.loginfo("Reaching Named Target Vertical...")
  #   success &= example.reach_named_position("vertical")
  #   print (success)
  
  # if success:
  #   rospy.loginfo("Reaching Joint Angles...")  
  #   success &= example.reach_joint_angles(tolerance=0.01) #rad
  #   print (success)
  
  # if success:
  #   rospy.loginfo("Reaching Named Target Home...")
  #   success &= example.reach_named_position("home")
  #   print (success)

  # if success:
  #   rospy.loginfo("Reaching Cartesian Pose...")
    
  #   actual_pose = example.get_cartesian_pose()
  #   actual_pose.position.z -= 0.2
  #   success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
  #   print (success)
    
  # if example.degrees_of_freedom == 7 and success:
  #   rospy.loginfo("Reach Cartesian Pose with constraints...")
  #   # Get actual pose
  #   actual_pose = example.get_cartesian_pose()
  #   actual_pose.position.y -= 0.3
    
  #   # Orientation constraint (we want the end effector to stay the same orientation)
  #   constraints = moveit_msgs.msg.Constraints()
  #   orientation_constraint = moveit_msgs.msg.OrientationConstraint()
  #   orientation_constraint.orientation = actual_pose.orientation
  #   constraints.orientation_constraints.append(orientation_constraint)

  #   # Send the goal
  #   success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

  # if example.is_gripper_present and success:
  #   rospy.loginfo("Opening the gripper...")
  #   success &= example.reach_gripper_position(0)
  #   print (success)

  #   rospy.loginfo("Closing the gripper 50%...")
  #   success &= example.reach_gripper_position(0.5)
  #   print (success)

  # # For testing purposes
  # rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  # if not success:
  #     rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()