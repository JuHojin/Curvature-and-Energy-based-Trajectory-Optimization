#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray

velocity_profile = []
velocity_constraints = []

def profile_callback(msg):
    global velocity_profile
    velocity_profile = msg.data

def constraints_callback(msg):
    global velocity_constraints
    velocity_constraints = msg.data

def plot_velocity_profile():
    plt.ion()
    plt.figure()

    while not rospy.is_shutdown():
        if velocity_profile and velocity_constraints:
            plt.clf()

            max_vel = velocity_constraints[:len(velocity_profile)]
            lat_acc = velocity_constraints[len(velocity_profile):]

            y_max = max(max_vel) * 1.2

            plt.plot(max_vel, "r-", label="max vel")
            plt.plot(lat_acc, "g-", label="lat acc")
            plt.plot(velocity_profile, "b-", label="result")
            plt.ylim(0,y_max)
            plt.title("Velocity Profiles")
            plt.xlabel("Path index")
            plt.ylabel("Speed [m/s]")
            plt.legend()
            plt.grid(True)

            plt.pause(0.1)

if __name__ == "__main__":
    rospy.init_node('velocity_profile_plotter', anonymous=True)

    rospy.Subscriber("/move_base/MyGlobalPlanner/velocity_profile", Float64MultiArray, profile_callback)
    rospy.Subscriber("/move_base/MyGlobalPlanner/velocity_constraints", Float64MultiArray, constraints_callback)
    
    plot_velocity_profile()

    rospy.spin()
