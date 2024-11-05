#!/usr/bin/env python

import rospy
import pandas as pd
import csv
import os
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Bool
from actionlib_msgs.msg import GoalID

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
POINTS_FILE = os.path.join(SCRIPT_DIR, '../data/start_goal_pair.csv')
TRAVEL_TIME_FILE = os.path.join(SCRIPT_DIR, '../data/travel_time.csv')
OPT_SUCCESS_FILE = os.path.join(SCRIPT_DIR, '../data/opt_success.csv')
SUCCESS_FILE = os.path.join(SCRIPT_DIR, '../data/success.csv')
CALCULATION_TIME_FILE = os.path.join(SCRIPT_DIR, '../data/calculation_time.csv')
CONSTANT_FILE = os.path.join(SCRIPT_DIR, '../../my_global_planner/include/constant.h')
START_EPSILON = 0.1

pub_command = rospy.Publisher('/car_1/command', AckermannDrive, queue_size = 1)
amcl_position = None
plan_received = False
planning_calculation_time = 0.0
optimization_success = False
calculation_time_list = []
optimization_success_list = []
success_list = []

def amcl_pose_callback(msg):
    global amcl_position
    amcl_position = msg.pose.pose.position

def latest_plan_callback(msg):
    global plan_received
    if msg:
        plan_received = True

def planning_calculation_time_callback(msg):
    global planning_calculation_time
    planning_calculation_time = msg.data
    calculation_time_list.append(planning_calculation_time)

def optimization_status_callback(msg):
    global optimization_success
    optimization_success = msg.data

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
            print("Start: x = {}, y = {}".format(x, y))
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
    print("Goal: x = {}, y = {}".format(x, y))

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
    # rospy.loginfo("Initial pose: x = {}, y = {}".format(x, y))

def cancel_move_base_goal():
    pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
    rospy.sleep(1)
    pub.publish(GoalID())

def parse_constants_from_h_file(file_path):
    if not os.path.exists(file_path):
        rospy.logerr("Constant.h file not found: {}".format(file_path))
        rospy.signal_shutdown("Required Constant.h file not found")
        return None

    constants = {}
    with open(file_path, 'r') as file:
        for line in file:
            if 'constexpr' in line and '=' in line:
                left_side, right_side = line.split('=', 1)
                
                key = left_side.strip().split(' ')[-1]
                value = right_side.strip().rstrip(';')
                try:
                    if '.' in value or 'e' in value:
                        value = float(value)
                    else:
                        value = int(value)
                except:
                    constants[key] = value

                constants[key] = value
    return constants

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
    
def save_data_to_csv(filename, path_planner, data_list):
    if os.path.exists(filename) and os.path.getsize(filename) != 0:
        df = pd.read_csv(filename)
    else:
        df = pd.DataFrame()
    
    df[path_planner] = pd.Series(data_list)
    df.to_csv(filename, index=False)
    print("Saved travel times of {} to {}".format(path_planner, filename))

def main():
    rospy.init_node('monte_carlo_runner', anonymous=True)


    # Initialize
    constants = parse_constants_from_h_file(CONSTANT_FILE)
    travel_time_list = []
    path_planner = rospy.get_param('~path_planner', 'QP')
    if path_planner == 'QP':
        gamma = constants.get('GAMMA')
        if not (gamma == 0 or gamma == 0.0):
            path_planner += '_E_' + str(gamma)
    
    # Subscribers
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback, queue_size=1)
    rospy.Subscriber('/move_base/MyGlobalPlanner/plan/latest', Path, latest_plan_callback, queue_size=1)
    rospy.Subscriber('/move_base/MyGlobalPlanner/planning_calculation_time', Float64, planning_calculation_time_callback, queue_size=1)
    if path_planner.startswith('QP'):
        rospy.Subscriber('/move_base/MyGlobalPlanner/optimization_status', Bool, optimization_status_callback, queue_size=1)

    # Set path planner parameter for global planner
    if path_planner.startswith('B_spline'):
        planner_type = 1
    elif path_planner.startswith('QP'):
        planner_type = 2
    else:
        planner_type = 0
    rospy.set_param('/move_base/planner_type', planner_type)
    rospy.loginfo("Set ROS parameter /planner_type to: {}".format(planner_type))

    # Load start and goal points
    start_points, goal_points = load_start_goal_points(POINTS_FILE)
    if start_points is None or goal_points is None:
        return

    # Monte Carlo Simulation
    for i in range(len(start_points)):
        print("""================================================\nplanner: {}, iteration: {}""".format(path_planner, i))
        
        # Reset ROS parameter - count_plan
        rospy.set_param('/move_base/count_plan', 0)
        rospy.loginfo("Set ROS parameter /move_base/count_plan to: 0")
        
        # Delete previous goal
        cancel_move_base_goal()
        
        start_x, start_y = start_points[i]
        goal_x, goal_y = goal_points[i]

        # Set start point
        call_reset_racecar_service(start_x, start_y)
        rospy.sleep(1)
        publish_initialpose(start_x, start_y)
        
        # Wait for AMCL to converge to start position
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if amcl_position is not None:
                distance = ((amcl_position.x - start_x) ** 2 + (amcl_position.y - start_y) ** 2) ** 0.5
                if distance < START_EPSILON:
                    break
            rate.sleep()

        # Set goal point
        publish_goal(goal_x, goal_y)
        rospy.sleep(1)
        
        # Wait for planning to complete
        global plan_received
        while not rospy.is_shutdown():
            if plan_received:
                rospy.set_param('/auto_control_active', True)
                start_time = rospy.Time.now()
                break
            rate.sleep()

        # Wait for car to arrive at goal
        successful_arrival = False
        while not rospy.is_shutdown():
            auto_control_active = rospy.get_param('/auto_control_active')
            count_plan = rospy.get_param('/move_base/count_plan')
            if count_plan == 1:
                successful_arrival = True
            elif count_plan > 1:
                travel_time = None
                print("Travel Time: {}".format(travel_time))
                # Car did not reach the goal successfully
                success_list.append(successful_arrival)
                break
                
            if not auto_control_active:
                # Travel time calculation
                end_time = rospy.Time.now()
                travel_time = (end_time - start_time).to_sec()
                travel_time_list.append(travel_time)
                print("Travel Time: {}".format(travel_time))

                # Car reached the goal successfully
                success_list.append(successful_arrival)
                break
            rate.sleep()

        # Stop the car
        command = AckermannDrive()
        command.steering_angle = 0
        command.speed = 0
        pub_command.publish(command)
        rospy.set_param('/auto_control_active', False)
        
        # Optimization success
        if path_planner.startswith('QP'):
            optimization_success_list.append(optimization_success)
            print("Optimization Success: {}".format(optimization_success))

        # Path planning calculation time
        print("Calculation Time: {}".format(calculation_time_list[i]))
        
        print("Arrived at Goal Successfuly: {}".format(successful_arrival))
        
        plan_received = False
        rospy.sleep(1.5)

    save_data_to_csv(TRAVEL_TIME_FILE, path_planner, travel_time_list)
    save_data_to_csv(CALCULATION_TIME_FILE, path_planner, calculation_time_list)
    save_data_to_csv(SUCCESS_FILE, path_planner, success_list)
    if path_planner.startswith('QP'):
        save_data_to_csv(OPT_SUCCESS_FILE, path_planner, optimization_success_list)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass