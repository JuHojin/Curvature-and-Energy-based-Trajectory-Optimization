#!/usr/bin/env python
import numpy as np
import os
import math
# import control as ctl
# import pandas as pd

import roslib
import rospy
import rospkg
import sys
import select
import termios
import tty
from scipy.linalg import solve_continuous_are

from std_msgs.msg import Int16
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped

import tf
from tf.transformations import euler_from_quaternion

# Normalize angle [-pi, +pi]
def normalize_angle(angle):
    if angle > math.pi:
        norm_angle = angle - 2*math.pi
    elif angle < -math.pi:
        norm_angle = angle + 2*math.pi
    else:
        norm_angle = angle
    return norm_angle

# Find nearest point
def find_nearest_point(ego_x, ego_y, x_list, y_list):
    x_arr = np.array(x_list)
    y_arr = np.array(y_list)

    distances = np.sqrt((x_arr - ego_x)**2 + (y_arr - ego_y)**2)

    near_idx = np.argmin(distances)
    near_dist = distances[near_idx]

    return near_idx, near_dist

# Calculate Error
def calc_error(ego_x, ego_y, ego_yaw, x_list, y_list, lookahead):
    # 2. Find the nearest waypoint
    near_idx, _ = find_nearest_point(ego_x, ego_y, x_list, y_list)
    rospy.loginfo("current near_idx: %s", near_idx)

    # 3. Set lookahead waypoint (index increment of waypoint trajectory)
    look_ahead_idx = max(min(near_idx + lookahead, len(x_list) - 1), 0)
    rospy.loginfo("current look_ahead_idx: %s", look_ahead_idx)

    lookahead_point_x = x_list[look_ahead_idx]
    lookahead_point_y = y_list[look_ahead_idx]

    # 4. Calculate errors
    if look_ahead_idx < len(x_list) - 1:
        error_yaw = math.atan2(y_list[look_ahead_idx + 1] - lookahead_point_y,
                               x_list[look_ahead_idx + 1] - lookahead_point_x) - ego_yaw
    else:
        error_yaw = math.atan2(lookahead_point_y - y_list[look_ahead_idx - 1],
                               lookahead_point_x - x_list[look_ahead_idx - 1]) - ego_yaw
    
    error_yaw = normalize_angle(error_yaw)  # Normalize angle to [-pi, +pi]
    
    gamma = math.atan2(y_list[look_ahead_idx] - ego_y, 
                       x_list[look_ahead_idx] - ego_x) - ego_yaw
    error_y = math.sqrt((x_list[look_ahead_idx] - ego_x) ** 2 + 
                        (y_list[look_ahead_idx] - ego_y) ** 2) * math.sin(gamma)

    return error_y, error_yaw

class AutoController():
    def __init__(self):
        # ROS init
        rospy.init_node('auto_control', anonymous = True)
        self.rate = rospy.Rate(10.0)

        # Params
        self.target_speed = 10/3.6 # 10km/h
        self.MAX_STEER    = np.deg2rad(17.75)

        # vehicle state
        self.ego_x   = 0
        self.ego_y   = 0
        self.ego_yaw = 0
        self.ego_vx  = 0

        self.wheelbase = 0.3

        self.wpt_look_ahead = 2   # [index]

        # Pub/Sub
        # self.sub_pose    = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback_pose)
        self.sub_path    = rospy.Subscriber('/move_base/MyGlobalPlanner/plan/uniform', Path, self.callback_path)
        self.sub_odom    = rospy.Subscriber('/car_1/base/odom', Odometry, self.callback_odom)
        self.pub_command = rospy.Publisher('/{}/command'.format(str(sys.argv[1])), AckermannDrive, queue_size = 1)

        # throttle PID
        self.integral_error_v = 0
        self.derivative_error_v = 0

    def callback_path(self, msg):
        self.x_list = []
        self.y_list = []

        for pose_stamped in msg.poses:
            self.x_list.append(pose_stamped.pose.position.x)
            self.y_list.append(pose_stamped.pose.position.y)

        if not self.x_list or not self.y_list:
            rospy.logwarn("Path is empty. Waiting for valid path...")
            return    

    def callback_odom(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        # get euler from quaternion
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        _, _, self.ego_yaw = euler_from_quaternion(q_list)

        # Get linear velocities in the global frame
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        # Convert global velocities to local velocities using the yaw angle
        self.ego_vx = vx * np.cos(self.ego_yaw) + vy * np.sin(self.ego_yaw)
        self.ego_vy = -vx * np.sin(self.ego_yaw) + vy * np.cos(self.ego_yaw)

        rospy.loginfo("self.ego_vx: %s", self.ego_vx)
        # Log the local velocities
        rospy.loginfo("Local linear velocities: vx: {:.3f}, vy: {:.3f}".format(self.ego_vx, self.ego_vy))

    # Controller
    def steer_control(self, error_y, error_yaw):
        L = 0.3
        v = self.ego_vx
        v = max(0.02, v)
        # rospy.loginfo("Control velocity (v): %s", v)

        A = np.array([[0, v],
                      [0, 0]])
        B = np.array([[0],
                      [v / L]])
        Q = np.array([[10, 0],
                      [0, 100]])
        R = np.array([[2500]])

        # rospy.loginfo("State matrix (A):\n%s\nControl matrix (B):\n%s", A, B)

        P = solve_continuous_are(A, B, Q, R)
        K = -np.dot(np.dot(np.linalg.inv(R), B.T), P)  

        # rospy.loginfo("Gain matrix (K):\n%s", K)

        steer = -np.dot(K, np.array([error_y, error_yaw]))
        steer = np.clip(steer, -self.MAX_STEER, self.MAX_STEER)
        # rospy.loginfo("error_y, error_yaw: %s, %s", error_y, error_yaw)
        # rospy.loginfo("Steering command before clipping: %s", steer)
        return steer

    def speed_control(self, error_v):
        Kp = 10
        Ki = 1
        Kd = 0.1

        dt = 0.01

        self.integral_error_v += error_v * dt

        if self.derivative_error_v is not None:
            derivative = (error_v - self.derivative_error_v) / dt
        else:
            derivative = 0

        throttle = Kp * error_v + Ki * self.integral_error_v + Kd * derivative

        self.derivative_error_v = error_v

        throttle = max(min(throttle, 1.0), -1.0)

        return throttle

    def publish_command(self, steer, speed):
        command = AckermannDrive()
        command.steering_angle = steer / np.deg2rad(17.75)
        command.speed = speed
        self.pub_command.publish(command)

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   rospy.loginfo("key input is %s" %(key))
   return key

def precompute_gain_tables(v_min, v_max, num_points):
    v_values = np.linspace(v_min, v_max, num_points)
    K_values = {}

    L = 0.3 # tmp wheel base
    Q = np.array([[10, 0],
                  [0, 100]])
    R = np.array([[2500]])

    for v in v_values:
        A = np.array([[0, v], [0, 0]])
        B = np.array([[0], [v / L]])
        P = solve_continuous_are(A, B, Q, R)
        K = -np.dot(np.dot(np.linalg.inv(R), B.T), P)
        K_values[v] = K

    return K_values

def main():

    # List to evaluate performance
    error_y_data = []
    error_v_data = []

    K_values = precompute_gain_tables(0.02, 4.02, 401)

    # Define controller
    auto_control = AutoController()
    try:
        while not rospy.is_shutdown():
            # key = getKey()
            # if key == '\x03':
            #     rospy.loginfo("Ctrl+C pressed. Exiting...")
            #     break

            if not hasattr(auto_control, 'x_list') or not auto_control.x_list:
                rospy.loginfo("Waiting for path data...")
                auto_control.rate.sleep()
                continue

            # Get current state
            ego_x = auto_control.ego_x - 9.0
            ego_y = auto_control.ego_y + 2.0
            ego_yaw = auto_control.ego_yaw
            ego_vx = auto_control.ego_vx
            wpts_x = auto_control.x_list
            wpts_y = auto_control.y_list

            # rospy.loginfo("Loaded waypoints: %d" % len(wpts_x))
            print("Loaded waypoints: %d, %d" % (len(wpts_x), len(wpts_y)))

            # Lateral error calculation (cross-track error, yaw error)
            error_y, error_yaw = calc_error(ego_x, ego_y, ego_yaw, wpts_x, wpts_y, lookahead=auto_control.wpt_look_ahead)

            # Longitudinal error calculation (speed error)
            # error_v = auto_control.target_speed - ego_vx

            # Log errors (errors w.r.t. the closest point)
            error_y_log, _ = calc_error(ego_x, ego_y, ego_yaw, wpts_x, wpts_y, lookahead=0)
            error_y_data.append(abs(error_y_log))
            # error_v_data.append(abs(error_v))

            # Control
            nearest_vx = min(K_values.keys(), key=lambda x: abs(x-ego_vx))
            K = np.array(K_values[nearest_vx])
            steer = -np.dot(K, np.array([error_y, error_yaw]))
            steer_cmd = np.clip(steer, -auto_control.MAX_STEER, auto_control.MAX_STEER)
            
            speed_cmd = auto_control.target_speed

            # Publish command
            auto_control.publish_command(steer_cmd, speed_cmd)

            # rospy.loginfo("Commands: (steer=%.3f, accel=%.3f). Errors: (CrossTrack=%.3f, Yaw=%.3f, Speed=%.3f). Ave. Errors ((CrossTrack=%.3f, Speed=%.3f)) ." %(steer_cmd, throttle_cmd, error_y, error_yaw, error_v, np.mean(error_y_data), np.mean(error_v_data)))
            rospy.loginfo("Steering command: %.3f, Speed command: %.3f" %(steer_cmd, speed_cmd))

            auto_control.rate.sleep()

    except Exception as e:
        rospy.logerr('Exception raised: {}'.format(e)) 

    finally:
        command = AckermannDrive()
        command.steering_angle = 0
        command.speed = 0
        auto_control.pub_command.publish(command)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    settings    = termios.tcgetattr(sys.stdin)
    main()