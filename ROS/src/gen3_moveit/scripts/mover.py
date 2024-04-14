from __future__ import print_function

import rospy

import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_commander import MoveGroupCommander
from copy import deepcopy
from gen3_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan
        
        
"""
    给定机器人的起始角度，规划一条以目标姿态结束的轨迹.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)

    return planCompat(move_group.plan())


def plan_trajectory_cartesian(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    waypoints = [destination_pose]  # 目标位置姿态作为路径的唯一路径点

    # 规划直线路径
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,  # 路径点
        0.01,       # 步长
        0.0         # 跳过碰撞检查
    )

    return plan  # 返回规划的路径


"""
    Creates a pick and place plan using the four states below.
    
    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position
    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved
    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""
def plan_pick_and_place(req):
    setup_scene()

    response = MoverServiceResponse()

    current_robot_joint_configuration = [
        math.radians(req.joints_input.joint_00),
        math.radians(req.joints_input.joint_01),
        math.radians(req.joints_input.joint_02),
        math.radians(req.joints_input.joint_03),
        math.radians(req.joints_input.joint_04),
        math.radians(req.joints_input.joint_05),
        math.radians(req.joints_input.joint_06),
    ]

    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(pre_grasp_pose)

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.075  # Static value coming from Unity, TODO: pass along with request
    grasp_pose = plan_trajectory_cartesian(move_group, pick_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(grasp_pose)

    # Pick Up - raise gripper back to the pre grasp position
    pick_up_pose = plan_trajectory_cartesian(move_group, req.pick_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(pick_up_pose)

    # Place - move gripper to desired placement position
    place_pose = plan_trajectory_cartesian(move_group, req.place_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = place_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(place_pose)

    # Post place - lift gripper after placement position
    place_pose = copy.deepcopy(req.place_pose)
    place_pose.position.z += 0.05 # Static value offset to lift up gripper
    post_place_pose = plan_trajectory_cartesian(move_group, place_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = post_place_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(post_place_pose)

    move_group.clear_pose_targets()

    return response

def setup_scene():
    global move_group
    
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    # Add table collider to MoveIt scene
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    rospy.sleep(2)

"""     p = PoseStamped()
    p.header.frame_id = move_group.get_planning_frame()
    p = set_pose(p, [0, 0, -0.02+0.75])
    scene.add_box("table", p, (1.2, 1.8, 0.01))
 """
def set_pose(poseStamped, pose):
    '''
    pose is an array: [x, y, z]
    '''
    poseStamped.pose.position.x = pose[0]
    poseStamped.pose.position.y = pose[1]
    poseStamped.pose.position.z = pose[2]
    return poseStamped

def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('gen3_moveit_server')
    cartesian = rospy.get_param('~cartesian',True)

    s = rospy.Service('gen3_moveit', MoverService, plan_pick_and_place)
    print("Ready to plan")

    rospy.spin()


if __name__ == "__main__":
    moveit_server()
