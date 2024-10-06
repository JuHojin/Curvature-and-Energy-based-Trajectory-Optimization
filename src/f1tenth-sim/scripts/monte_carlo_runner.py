#!/usr/bin/env python

import rospy
import random
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path

START_MIN_X = -12.0
START_MAX_X = -10.0
START_MIN_Y = -8.0
START_MAX_Y = 8.8

GOAL_MIN_X = 11.9
GOAL_MAX_X = 12.1
GOAL_MIN_Y = -8.0
GOAL_MAX_Y = 8.8

amcl_position = None
plan_received = False

def amcl_pose_callback(msg):
    global amcl_position
    amcl_position = msg.pose.pose.position

def latest_plan_callbacK(msg):
    global plan_received
    plan_received = True

def get_random_position(min_x, max_x, min_y, max_y):
    return random.uniform(min_x, max_x), random.uniform(min_y, max_y)

def call_reset_racecar_service(x, y):
    rospy.wait_for_service('/reset_racecar_pose')
    try:
        rest_racecar = rospy.ServiceProxy('/reset_racecar_pose', SetModelState)

        request = SetModelStateRequest()
        request.model_state.model_name = 'car_1'
        request.model_state.pose.position.x = x
        request.model_state.pose.position.y = y
        request.model_state.pose.position.z = 0.001
        request.model_state.pose.orientation.w = 1.0

        response = rest_racecar(request)
        if response.success:
            rospy.loginfo("Car position successfully reset: ({}, {})".format(x, y))
        else:
            rospy.logerr("Failed to reset car position")
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def publish_goal(x, y):
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    
    goal_msg = PoseStamped()
    goal_msg.header.frame_id = "map"
    goal_msg.header.stamp = rospy.Time.now()

    goal_msg.pose.position.x = x
    goal_msg.pose.position.y = y
    goal_msg.pose.position.z = 0.0

    goal_msg.pose.orientation.x = 0.0
    goal_msg.pose.orientation.y = 0.0
    goal_msg.pose.orientation.z = 0.0
    goal_msg.pose.orientation.w = 1.0

    rospy.sleep(1)
    pub.publish(goal_msg)
    rospy.loginfo("Goal published at: x = {}, y = {}".format(x, y))

def publish_initialpose(x, y):
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.frame_id = 'map'
    pose_msg.header.stamp = rospy.Time.now()

    pose_msg.pose.pose.position.x = x
    pose_msg.pose.pose.position.y = y
    pose_msg.pose.pose.position.z = 0.001
    pose_msg.pose.pose.orientation.x = 0.0
    pose_msg.pose.pose.orientation.y = 0.0
    pose_msg.pose.pose.orientation.z = 0.0
    pose_msg.pose.pose.orientation.w = 1.0

    pose_msg.pose.covariance = [0.0]*36
    pose_msg.pose.covariance[0] = 0.5
    pose_msg.pose.covariance[7] = 0.5
    pose_msg.pose.covariance[14] = 0.1
    pose_msg.pose.covariance[21] = 0.1
    pose_msg.pose.covariance[28] = 0.1
    pose_msg.pose.covariance[35] = 0.1

    rospy.sleep(1)
    pub.publish(pose_msg)
    rospy.loginfo("Initial pose: x = {}, y = {}".format(x, y))


def main():
    rospy.init_node('monte_carlo_runner', anonymous=True)
    
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.Subscriber('/move_base/MyGlobalPlanner/plan/latest', Path, latest_plan_callbacK)

    iterations = rospy.get_param("~iterations", 100)

    for i in range(iterations):
        # Auto-control disable
        rospy.set_param('/auto_control_active', False)

        start_x, start_y = get_random_position(START_MIN_X, START_MAX_X, START_MIN_Y, START_MAX_Y)
        goal_x, goal_y = get_random_position(GOAL_MIN_X, GOAL_MAX_X, GOAL_MIN_Y, GOAL_MAX_Y)
        
        rospy.loginfo("start: ({}, {}), goal ({}, {})".format(start_x, start_y, goal_x, goal_y))
        call_reset_racecar_service(start_x, start_y)
        rospy.sleep(1)
        publish_initialpose(start_x, start_y)
        
        # Wait for AMCL to converge to start position
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if amcl_position is not None:
                distance = ((amcl_position.x - start_x) ** 2 + (amcl_position.y - start_y) ** 2) ** 0.5
                if distance < 0.01:
                    break
            rate.sleep()

        # Set goal point
        publish_goal(goal_x, goal_y)

        while not rospy.is_shutdown():
            if plan_received:
                rospy.set_param('/auto_control_active', True)
                break
            rate.sleep()

        # Wait for car to arrive at goal
        while not rospy.is_shutdown():
            if amcl_position is not None:
                distance = ((amcl_position.x - goal_x) ** 2 + (amcl_position.y - goal_y) ** 2) ** 0.5
                if distance < 0.01:
                    break
            rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
