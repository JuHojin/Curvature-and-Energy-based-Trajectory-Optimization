#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from control_msgs.msg import JointControllerState

import threading

velocity_profile = []
velocity_constraints = []

actual_velocity = 0
speed_command = 0
steering_command = 0
left_steering = 0
left_front_command = 0
left_rear_command = 0
left_front_state = 0
left_rear_state = 0

def left_steering_state_callback(msg):
    global left_steering    
    left_steering = msg.process_value


def left_front_callback(msg):
    global left_front_command
    global left_front_state
    left_front_command = 0.05 * msg.set_point
    left_front_state = 0.05 * msg.process_value

def left_rear_callback(msg):
    global left_rear_state
    global left_rear_command
    left_rear_command = 0.05 * msg.set_point
    left_rear_state = 0.05 * msg.process_value


def velocity_callback(msg):
    global actual_velocity
    vel_x = msg.twist.twist.linear.x
    vel_y = msg.twist.twist.linear.y
    actual_velocity = (vel_x ** 2 + vel_y ** 2) ** 0.5
    rospy.loginfo("Actual velocity: %f", actual_velocity)

def command_callback(msg):
    global speed_command
    global steering_command
    speed_command = msg.speed
    steering_command = msg.steering_angle

def profile_callback(msg):
    global velocity_profile
    velocity_profile = msg.data

def constraints_callback(msg):
    global velocity_constraints
    velocity_constraints = msg.data

def plot_actual_velocity():
    plt.ion()  # Interactive 모드 활성화
    plt.figure(1)  # 첫 번째 플롯 창 열기
    actual_velocity_plot = []  # 실제 속도 값을 저장할 리스트
    speed_command_plot = []
    left_front_command_plot = []
    left_front_state_plot = []
    left_rear_command_plot = []
    left_rear_state_plot = []

    plt.show(block=False)  # 비차단 모드로 창 열기

    while not rospy.is_shutdown():
        plt.clf()  # 그래프 초기화

        # 실제 속도 리스트 업데이트
        actual_velocity_plot.append(actual_velocity)
        speed_command_plot.append(speed_command)
        left_front_command_plot.append(left_front_command)
        left_front_state_plot.append(left_front_state)
        left_rear_command_plot.append(left_rear_command)
        left_rear_state_plot.append(left_rear_state)
        
        # 실제 속도 플롯
        plt.plot(actual_velocity_plot, "m-", label="actual velocity")
        plt.plot(speed_command_plot, label="speed_command_plot")
        # plt.plot(left_front_command_plot, label="left_front_command_plot")
        # plt.plot(left_front_state_plot, label="left_front_state_plot")
        # plt.plot(left_rear_command_plot, label="left_rear_command_plot")
        # plt.plot(left_rear_state_plot,label="left_rear_state_plot")
        
        # y축 범위 설정 (상황에 맞게 조정)
        if speed_command_plot:
            y_min = min(speed_command_plot) * 1.2
            y_max = max(speed_command_plot) * 1.2
            plt.ylim(y_min, y_max)
        else:
            plt.ylim(-1, 1)

        # 그래프 타이틀 및 레이블
        plt.title("Actual Velocity")
        plt.xlabel("Time Step")
        plt.ylabel("Speed [m/s]")
        plt.legend()
        plt.grid(True)

        # 그래프 갱신
        plt.pause(0.1)


def plot_velocity_profile():
    plt.ion()  # Interactive 모드 활성화
    plt.figure(2)  # 두 번째 플롯 창 열기

    plt.show(block=False)  # 비차단 모드로 창 열기

    while not rospy.is_shutdown():
        if velocity_profile and velocity_constraints:
            plt.clf()

            max_vel = velocity_constraints[:len(velocity_profile)]
            lat_acc = velocity_constraints[len(velocity_profile):]

            if max_vel:
                y_min = min(max_vel) * 1.2
                y_max = max(max_vel) * 1.2
                plt.ylim(y_min, y_max)
            else:
                plt.ylim(-1, 1)

            if lat_acc:
                lat_acc = max(lat_acc, max(max_vel) * 1.2)

            plt.plot(max_vel, "r-", label="max vel")
            plt.plot(lat_acc, "g-", label="lat acc")
            plt.plot(velocity_profile, "b-", label="result")
            plt.ylim(0, y_max)
            plt.title("Velocity Profiles")
            plt.xlabel("Path index")
            plt.ylabel("Speed [m/s]")
            plt.legend()
            plt.grid(True)

            plt.pause(0.1)

def plot_actual_steering():
    plt.ion()  # Interactive 모드 활성화
    plt.figure(3) 
    actual_steering_plot = []  # 실제 속도 값을 저장할 리스트
    steering_command_plot = []

    plt.show(block=False)  # 비차단 모드로 창 열기

    while not rospy.is_shutdown():
        plt.clf()  # 그래프 초기화

        # 실제 속도 리스트 업데이트
        actual_steering_plot.append(left_steering)
        steering_command_plot.append(steering_command)
        
        # 실제 속도 플롯
        plt.plot(actual_steering_plot, "m-", label="actual_left_steering")
        plt.plot(steering_command_plot, label="steering_command")
        
        # y축 범위 설정 (상황에 맞게 조정)
        if steering_command_plot:
            y_min = min(steering_command_plot) * 1.2
            y_max = max(steering_command_plot) * 1.2
            plt.ylim(y_min, y_max)
        else:
            plt.ylim(-1, 1)

        # 그래프 타이틀 및 레이블
        plt.title("steering")
        plt.xlabel("Time Step")
        plt.ylabel("Rad")
        plt.legend()
        plt.grid(True)

        # 그래프 갱신
        plt.pause(0.1)

if __name__ == "__main__":
    rospy.init_node('velocity_profile_plotter', anonymous=True)

    # 메시지 구독 설정
    rospy.Subscriber("/move_base/MyGlobalPlanner/velocity_profile", Float64MultiArray, profile_callback)
    rospy.Subscriber("/move_base/MyGlobalPlanner/velocity_constraints", Float64MultiArray, constraints_callback)
    rospy.Subscriber("/car_1/base/odom", Odometry, velocity_callback)  # Odometry 메시지 구독
    rospy.Subscriber('/car_1/command', AckermannDrive, command_callback)
    rospy.Subscriber("/car_1/left_steering_hinge_position_controller/state", JointControllerState, left_steering_state_callback)

    rospy.Subscriber("/car_1/left_front_wheel_velocity_controller/state", JointControllerState, left_front_callback)
    rospy.Subscriber("/car_1/left_rear_wheel_velocity_controller/state", JointControllerState, left_rear_callback)

    # 두 개의 함수를 멀티스레딩으로 실행
    # thread1 = threading.Thread(target=plot_actual_velocity)
    thread2 = threading.Thread(target=plot_velocity_profile)
    # thread3 = threading.Thread(target=plot_actual_steering)

    # thread1.start()
    thread2.start()
    # thread3.start()

    rospy.spin()
