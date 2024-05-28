#!/usr/bin/env python3

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

from gen3_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

class ExampleMoveItTrajectories(object):
  """MoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
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


  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # 实现其中一个目标
    rospy.loginfo("Going to named target " + target)
    # 设定目标
    arm_group.set_named_target(target)
    # 规划轨迹
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # 执行轨迹并在未完成时阻止
    return arm_group.execute(trajectory_message, wait=True)

  def reach_joint_angles(self, tolerance):
    arm_group = self.arm_group
    success = True

    # 获取当前关节位置
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # 设定目标关节容差
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # 设置联合目标配置
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
    
    # 在一个命令中计划和执行
    success &= arm_group.go(wait=True)

    # 显示运动后的关节位置
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # 获取当前姿势并显示
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # 设置容差
    arm_group.set_goal_position_tolerance(tolerance)

    # 如果指定了轨迹约束，则设置轨迹约束
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # 获取当前笛卡尔位置
    arm_group.set_pose_target(pose)

    # 计划和执行
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # 我们只需要移动这个关节，因为所有其他关节都是模仿的！
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 
    

def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('gen3_moveit_server')
    cartesian = rospy.get_param('~cartesian',True)

    s = rospy.Service('gen3_moveit', MoverService, plan_pick_and_place)
    print("Ready to plan")

    rospy.spin()


def plan_pick_and_place(req):
  
  example = ExampleMoveItTrajectories()
  rospy.loginfo("plan pick and play is ready")
  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass

  """ if success:
    rospy.loginfo("Reaching Named Target Vertical...")
    success &= example.reach_named_position("vertical")
    print (success)
  
  if success:
    rospy.loginfo("Reaching Joint Angles...")  
    success &= example.reach_joint_angles(tolerance=0.1) #rad
    print (success) """
  
  if success:
    rospy.loginfo("Reaching Named Target Home...")
    success &= example.reach_named_position("home")
    print (success)

  if success:
    rospy.loginfo("Reaching Cartesian Pose1...")
    
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.x =req.pick_pose.position.x
    actual_pose.position.y =req.pick_pose.position.y
    actual_pose.position.z =req.pick_pose.position.z
    actual_pose.orientation.x = 0
    actual_pose.orientation.y = 1
    actual_pose.orientation.z = 0
    actual_pose.orientation.w = 0
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    rospy.loginfo("pose1 zitai")
    actual_pose = example.get_cartesian_pose()
    print (success)

  if success:
    rospy.loginfo("Reaching Cartesian Pose2...")
    
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.x =0.5
    actual_pose.position.y =0.4
    actual_pose.position.z =0.1
    rospy.loginfo("pose2 zitai")

    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)
    
  if example.degrees_of_freedom == 7 and success:
    rospy.loginfo("Reach Cartesian Pose with constraints...")
    # 获取实际姿势
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.z -= 0.05
    
    # 方向约束（我们希望末端执行器保持相同的方向)
    constraints = moveit_msgs.msg.Constraints()
    orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    orientation_constraint.orientation = actual_pose.orientation
    constraints.orientation_constraints.append(orientation_constraint)

    # 发送目标
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

  if example.is_gripper_present and success:
    rospy.loginfo("Opening the gripper...")
    success &= example.reach_gripper_position(0)
    print (success)

    rospy.loginfo("Closing the gripper 50%...")
    success &= example.reach_gripper_position(0.5)
    print (success)

  # 测试
  rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  if not success:
      rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  moveit_server()
