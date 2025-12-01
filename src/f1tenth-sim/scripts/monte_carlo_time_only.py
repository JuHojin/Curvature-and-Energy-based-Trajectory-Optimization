#!/usr/bin/env python

import rospy
import pandas as pd
import csv
import os
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Bool
from actionlib_msgs.msg import GoalID

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(SCRIPT_DIR)
POINTS_FILE = os.path.join(PARENT_DIR, 'data', 'start_goal_pair.csv')
CALCULATION_TIME_FILE = os.path.join(PARENT_DIR, 'data', 'calculation_time.csv')
OPT_SUCCESS_FILE = os.path.join(PARENT_DIR, 'data', 'opt_success.csv')
CONSTANT_FILE = os.path.join(SCRIPT_DIR, '../../my_global_planner/include/constant.h')
START_EPSILON = 0.1
PLAN_TIMEOUT_SEC = 30.0

amcl_position = None
plan_received = False
latest_plan_goal = None
planning_calculation_time = float('nan')
optimization_success = False


def amcl_pose_callback(msg):
    global amcl_position
    amcl_position = msg.pose.pose.position


def latest_plan_callback(msg):
    global plan_received
    global latest_plan_goal
    if msg and msg.poses:
        plan_received = True
        latest_plan_goal = msg.poses[-1].pose


def planning_calculation_time_callback(msg):
    global planning_calculation_time
    planning_calculation_time = msg.data


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

        request.model_state.twist.linear.x = 0.0
        request.model_state.twist.linear.y = 0.0
        request.model_state.twist.linear.z = 0.0
        request.model_state.twist.angular.x = 0.0
        request.model_state.twist.angular.y = 0.0
        request.model_state.twist.angular.z = 0.0

        response = rest_racecar(request)
        if response.success:
            rospy.loginfo("Start reset: x = %.3f, y = %.3f", x, y)
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

    rospy.sleep(1.0)
    pub.publish(goal_msg)
    rospy.loginfo("Goal published: x = %.3f, y = %.3f", x, y)


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

    pose_msg.pose.covariance = [0.0] * 36
    pose_msg.pose.covariance[0] = 0.5
    pose_msg.pose.covariance[7] = 0.5
    pose_msg.pose.covariance[14] = 0.1
    pose_msg.pose.covariance[21] = 0.1
    pose_msg.pose.covariance[28] = 0.1
    pose_msg.pose.covariance[35] = 0.1

    rospy.sleep(1.0)
    pub.publish(pose_msg)


def cancel_move_base_goal():
    pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
    rospy.sleep(1.0)
    pub.publish(GoalID())


def parse_constants_from_h_file(file_path):
    if not os.path.exists(file_path):
        rospy.logerr("constant.h file not found: {}".format(file_path))
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
                except ValueError:
                    constants[key] = value
                    continue

                constants[key] = value
    return constants


def load_start_goal_points(filename):
    if os.path.exists(filename):
        with open(filename, 'r') as file:
            csv_reader = csv.reader(file)
            next(csv_reader)  # skip the header
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


def load_existing_data(filename, path_planner):
    if os.path.exists(filename) and os.path.getsize(filename) != 0:
        try:
            df = pd.read_csv(filename, usecols=[path_planner])
            return df[path_planner].dropna().tolist()
        except ValueError:
            rospy.logwarn("Column %s not found in %s, returning empty list.", path_planner, filename)
            return []
    else:
        return []


def save_data_to_csv(filename, path_planner, data_list):
    if not os.path.exists(filename) or os.path.getsize(filename) == 0:
        df = pd.DataFrame({path_planner: data_list})
    else:
        df = pd.read_csv(filename)

        if path_planner in df.columns:
            existing_length = len(df[path_planner])
            new_length = len(data_list)

            if new_length > existing_length:
                df.loc[:existing_length - 1, path_planner] = data_list[:existing_length]
                df = pd.concat(
                    [df, pd.DataFrame({path_planner: data_list[existing_length:]})],
                    ignore_index=True,
                    sort=False
                )
            else:
                df.loc[:new_length - 1, path_planner] = data_list
        else:
            df[path_planner] = pd.Series(data_list)

    df.to_csv(filename, index=False)


def save_iteration_data(path_planner, calculation_time_list, optimization_success_list=None):
    save_data_to_csv(CALCULATION_TIME_FILE, path_planner, calculation_time_list)
    if path_planner.startswith('QP') and optimization_success_list is not None:
        save_data_to_csv(OPT_SUCCESS_FILE, path_planner, optimization_success_list)
    rospy.loginfo("Calculation-time data saved")


def wait_for_amcl(start_x, start_y):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if amcl_position is not None:
            distance = ((amcl_position.x - start_x) ** 2 + (amcl_position.y - start_y) ** 2) ** 0.5
            if distance < START_EPSILON:
                return True
        rate.sleep()
    return False


def main():
    rospy.init_node('monte_carlo_time_runner', anonymous=True)

    constants = parse_constants_from_h_file(CONSTANT_FILE)
    if constants is None:
        return

    path_planner = rospy.get_param('~path_planner', 'QP')
    if path_planner == 'QP':
        gamma = constants.get('GAMMA')
        if gamma not in (0, 0.0, None):
            path_planner += '_E_' + str(gamma)

    calculation_time_list = load_existing_data(CALCULATION_TIME_FILE, path_planner)
    optimization_success_list = load_existing_data(OPT_SUCCESS_FILE, path_planner) if path_planner.startswith('QP') else []

    if path_planner.startswith('QP'):
        min_length = min(
            len(pd.Series(calculation_time_list).dropna()),
            len(pd.Series(optimization_success_list).dropna())
        ) - 1
    else:
        min_length = len(pd.Series(calculation_time_list).dropna()) - 1

    resume_idx = max(0, min_length)
    calculation_time_list = calculation_time_list[:resume_idx]
    optimization_success_list = optimization_success_list[:resume_idx]
    rospy.loginfo("Resuming Monte Carlo time-only run from index %d", resume_idx)

    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback, queue_size=1)
    rospy.Subscriber('/move_base/MyGlobalPlanner/plan/latest', Path, latest_plan_callback, queue_size=1)
    rospy.Subscriber('/move_base/MyGlobalPlanner/planning_calculation_time', Float64, planning_calculation_time_callback, queue_size=1)
    if path_planner.startswith('QP'):
        rospy.Subscriber('/move_base/MyGlobalPlanner/optimization_status', Bool, optimization_status_callback, queue_size=1)

    if path_planner.startswith('B_spline'):
        planner_type = 1
    elif path_planner.startswith('QP'):
        planner_type = 2
    else:
        planner_type = 0
    rospy.set_param('/move_base/planner_type', planner_type)
    rospy.loginfo("Set /move_base/planner_type to %d", planner_type)

    start_points, goal_points = load_start_goal_points(POINTS_FILE)
    if start_points is None or goal_points is None:
        return

    global plan_received
    global latest_plan_goal
    global planning_calculation_time

    for i in range(resume_idx, len(start_points)):
        rospy.loginfo("================================================")
        rospy.loginfo("planner: %s, iteration: %d", path_planner, i + 1)

        cancel_move_base_goal()

        start_x, start_y = start_points[i]
        goal_x, goal_y = goal_points[i]

        call_reset_racecar_service(start_x, start_y)
        rospy.sleep(1.0)
        publish_initialpose(start_x, start_y)

        if not wait_for_amcl(start_x, start_y):
            break

        rospy.set_param('/move_base/count_plan', 0)
        rospy.set_param('/auto_control_active', False)

        plan_received = False
        latest_plan_goal = None
        planning_calculation_time = float('nan')

        publish_goal(goal_x, goal_y)
        rospy.sleep(1.0)
        rospy.loginfo("Waiting for plan result...")

        planning_timeout = rospy.Duration(PLAN_TIMEOUT_SEC)
        wait_start = rospy.Time.now()
        plan_completed = False

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if plan_received and latest_plan_goal is not None:
                if abs(latest_plan_goal.position.x - goal_x) < 0.1 and abs(latest_plan_goal.position.y - goal_y) < 0.1:
                    plan_completed = True
                    break
            if rospy.Time.now() - wait_start > planning_timeout:
                rospy.logwarn("Planning timeout for iteration %d", i + 1)
                break
            rate.sleep()

        cancel_move_base_goal()

        if plan_completed:
            calculation_time_list.append(planning_calculation_time)
            rospy.loginfo("Calculation Time: %.6f s", planning_calculation_time)
            if path_planner.startswith('QP'):
                optimization_success_list.append(optimization_success)
                rospy.loginfo("Optimization Success: %s", optimization_success)
        else:
            calculation_time_list.append(float('nan'))
            if path_planner.startswith('QP'):
                optimization_success_list.append(False)

        rospy.sleep(1.5)
        save_iteration_data(path_planner, calculation_time_list, optimization_success_list)

    rospy.loginfo("Monte Carlo time-only run completed")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
