#!/usr/bin/env python

import rospy
import csv
import os
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalID

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
POINTS_FILE = os.path.join(SCRIPT_DIR, '../data/start_goal_pair.csv')

amcl_position = None
plan_received = False

def amcl_pose_callback(msg):
    global amcl_position
    amcl_position = msg.pose.pose.position

def latest_plan_callback(msg):
    global plan_received
    if msg:
        plan_received = True

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

def load_start_goal_points(filename):
    if os.path.exists(filename):
        with open(filename, 'r') as file:
            csv_reader = csv.reader(file)
            next(csv_reader) # skip the header
            start_points = []
            goal_points = []
            for row in csv_reader:
                start_points.append((float(row[0]), float(row[1])))
                goal_points.append((float(row[2]), float(row[3])))
        return start_points, goal_points
    else:
        rospy.logerr("Points file not found: {}".format(filename))
        rospy.signal_shutdown("Required start-goal points file not found")
        return None, None
    
def cancel_move_base_goal():
    pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
    rospy.sleep(1)
    pub.publish(GoalID())
    rospy.loginfo("Cancelled previous move_base goal")


def main():
    rospy.init_node('monte_carlo_runner', anonymous=True)
    
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback, queue_size=1)
    rospy.Subscriber('/move_base/MyGlobalPlanner/plan/latest', Path, latest_plan_callback, queue_size=1)

    start_points, goal_points = load_start_goal_points(POINTS_FILE)
    if start_points is None or goal_points is None:
        return

    for i in range(len(start_points)):
        # Auto-control disable
        rospy.set_param('/auto_control_active', False)
        
        start_x, start_y = start_points[i]
        goal_x, goal_y = goal_points[i]

        rospy.loginfo("start: ({}, {}), goal ({}, {})".format(start_x, start_y, goal_x, goal_y))
        call_reset_racecar_service(start_x, start_y)
        rospy.sleep(1)
        publish_initialpose(start_x, start_y)
        
        # Wait for AMCL to converge to start position
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if amcl_position is not None:
                distance = ((amcl_position.x - start_x) ** 2 + (amcl_position.y - start_y) ** 2) ** 0.5
                if distance < 0.1:
                    break
            rate.sleep()

        # Set goal point
        publish_goal(goal_x, goal_y)
        rospy.sleep(1)
        
        global plan_received
        while not rospy.is_shutdown():
            print("plan_received = {}".format(plan_received))
            if plan_received:
                rospy.set_param('/auto_control_active', True)
                plan_received = False
                break
            rate.sleep()

        # Wait for car to arrive at goal
        while not rospy.is_shutdown():
            if amcl_position is not None:
                distance = ((amcl_position.x - goal_x) ** 2 + (amcl_position.y - goal_y) ** 2) ** 0.5
                if distance < 10:
                    print(distance)
                if distance < 0.7:
                    cancel_move_base_goal()
                    break
            rate.sleep()

        rospy.loginfo("Racecar arrived at the goal")
        # rospy.sleep(2)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
