#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def main():
    rclpy.init()
    nav = BasicNavigator()

    # set initial pose
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.pose.position.x = 0.0
    initial_pose.pose.pose.position.y = 0.0
    initial_pose.pose.pose.position.z = 0.0
    initial_pose.pose.pose.orientation.x = q_x
    initial_pose.pose.pose.orientation.y = q_y
    initial_pose.pose.pose.orientation.z = q_z
    initial_pose.pose.pose.orientation.w = q_w


    nav.setInitialPose(initial_pose)

    # wait for Nav2
    nav.waitUntilNav2Active()

    # send Nav2 goal
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 1.57)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    nav.goToPose(goal_pose)

    # Monitor progress until the goal is reached
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)

    print(nav.getResult())

    # shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
