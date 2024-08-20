#!/usr/bin/env python
import numpy as np
import os
import math

import roslib
import rospy
import rospkg

from std_msgs.msg import Int16
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped

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

def find_nearest_point(ego_x, ego_y, x_list, y_list):

    x_arr = np.array(x_list)
    y_arr = np.array(y_list)

    distances = np.sqrt((x_arr - ego_x)**2 + (y_arr - ego_y)**2)

    near_idx = np.argmin(distances)
    near_dist = distances[near_idx]

    return near_idx, near_dist

# Calculate Error
def calc_error(ego_x, ego_y, ego_yaw, x_list, y_list, lookahead=1):
    """
    TODO
    Local coordinate approach
    (!It doesn't matter if you implement the global coordinates approach.)
    (!Just return the cross-track and heading errors.)
    1. Transform from global to local coordinate trajectory.
    2. Find the nearest waypoint.
    3. Set lookahead waypoint.
        - (hint) use the index of the nearest waypoint (near_ind) from 'find_nearest_point'.
        - (hint) consider that the next index of the terminal waypoint is 0, which is not terminal_index + 1.
    4. Calculate errors
        - error_yaw (yaw error)
            : (hint) use math.atan2 for calculating angle btw lookahead and next waypoints.
            : (hint) consider that the next index of the terminal waypoint is 0, which is not terminal_index + 1.
        - error_y (crosstrack error)
            : (hint) y value of the lookahead point in local coordinate waypoint trajectory.
    """
    # # 1. Global to Local coordinate
    # local_x_list, local_y_list = global2local(ego_x, ego_y, ego_yaw, x_list, y_list)

    # 2. Find the nearest waypoint
    _, near_idx = find_nearest_point(ego_x, ego_y, x_list, y_list)
    nearest_point_x = x_list[near_idx]
    nearest_point_y = y_list[near_idx]

    # 3. Set lookahead waypoint (index increment of waypoint trajectory)
    lookahead_idx = near_idx + lookahead

    if look_ahead_idx > len(x_list) - 1:
            look_ahead_idx = len(x_list) - 1
            
    lookahead_point_x = x_list[lookahead_idx]
    loodahead_point_y = y_list[lookahead_idx]

    # 4. Calculate errors
    error_yaw = 0
    error_yaw = normalize_angle(error_yaw) # Normalize angle to [-pi, +pi]
    error_y   = 0
    
    return error_y, error_yaw

class Auto_Controller():
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

        self.wpt_look_ahead = 0   # [index]

        # Pub/Sub
        self.sub_pose    = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback_pose)
        self.sub_path    = rospy.Subscriber('/move_base/MyGlobalPlanner/plan/uniform', Path, self.callback_path)
        self.pub_command = rospy.Publisher('/control', AckermannDrive, queue_size=5)

        rospy.spin()

    def callback_pose(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y

        # get euler from quaternion
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        _, _, self.ego_yaw = euler_from_quaternion(q_list)

    def callback_path(self, msg):
        x_list = []
        y_list = []

        for pose_stamped in msg.poses:
            x_list.append(pose_stamped.pose.position.x)
            y_list.append(pose_stamped.pose.position.y)
        rospy.loginfo("X coordinates: %s", x_list)
        rospy.loginfo("Y coordinates: %s", y_list)            

    # Controller
    def steer_control(self, error_y, error_yaw):
        """
        TODO
        Implement a steering controller (PID controller or Pure pursuit or Stanley method).
        You can use not only error_y, error_yaw, but also other input arguments for this controller if you want.
        """
        steer = 0

        # Control limit
        steer = np.clip(steer, -self.MAX_STEER, self.MAX_STEER)

        return steer

    def speed_control(self, error_v):
        """
        TODO
        Implement a speed controller (PID controller).
        You can use not only error_v, but also other input arguments for this controller if you want.
        """
        throttle = 0
                
        return throttle

    def publish_command(self, steer, accel):
        """
        Publish command as AckermannDriveStamped
        ref: http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html
        """
        msg = AckermannDrive()
        msg.drive.steering_angle = steer / np.deg2rad(17.75)
        msg.drive.acceleration = accel
        self.pub_command.publish(msg)

def main():
    # Load Waypoint
    rospack = rospkg.RosPack()
    WPT_CSV_PATH = rospack.get_path('waypoint_follower') + "/wpt_data/wpt_data_dense.csv"
    csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
    wpts_x = csv_data.values[:,0]
    wpts_y = csv_data.values[:,1]
    
    # List to evaluate performance
    error_y_data = []
    error_v_data = []

    print("loaded wpt :", wpts_x.shape, wpts_y.shape)

    # Define controller
    wpt_control = WaypointFollower()

    while not rospy.is_shutdown():
        # Get current state
        ego_x = wpt_control.ego_x
        ego_y = wpt_control.ego_y
        ego_yaw = wpt_control.ego_yaw
        ego_vx = wpt_control.ego_vx

        # Lateral error calculation (cross-track error, yaw error)
        error_y, error_yaw = calc_error(ego_x, ego_y, ego_yaw, wpts_x, wpts_y, wpt_lookahead=wpt_control.wpt_look_ahead)

        # Longitudinal error calculation (speed error)
        error_v = wpt_control.target_speed - ego_vx

        # Log errors (errors w.r.t. the closest point)
        error_y_log, _ = calc_error(ego_x, ego_y, ego_yaw, wpts_x, wpts_y, wpt_look_ahead=0)
        error_y_data.append(abs(error_y_log))
        error_v_data.append(abs(error_v))

        # Control
        steer_cmd = wpt_control.steer_control(error_y, error_yaw)
        throttle_cmd = wpt_control.speed_control(error_v)

        # Publish command
        wpt_control.publish_command(steer_cmd, throttle_cmd)

        rospy.loginfo("Commands: (steer=%.3f, accel=%.3f). Errors: (CrossTrack=%.3f, Yaw=%.3f, Speed=%.3f). Ave. Errors ((CrossTrack=%.3f, Speed=%.3f)) ." %(steer_cmd, throttle_cmd, error_y, error_yaw, error_v, np.mean(error_y_data), np.mean(error_v_data)))
        wpt_control.rate.sleep()


if __name__ == '__main__':
    main()