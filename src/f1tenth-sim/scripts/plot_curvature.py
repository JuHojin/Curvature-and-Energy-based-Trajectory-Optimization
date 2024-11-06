#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import threading

class CurvaturePlotter:
    def __init__(self):
        rospy.init_node('curvature_plotter', anonymous=True)

        plt.ion()
        # 여러 subplot을 생성하여 데이터를 각각 플로팅할 준비
        self.fig, (self.ax_peaks, self.ax_sums, self.ax_times) = plt.subplots(3, 1, figsize=(12, 18))

        # 각 subplot에 대한 초기 설정
        self.line_peaks, = self.ax_peaks.plot([], [], color='navy', label='Curvature Peaks', linewidth=5.0)  # 선 굵기 설정
        # self.ax_peaks.set_xlabel('Iteration', fontsize=20)  # x축 글씨 크기 설정
        self.ax_peaks.set_ylabel('Curvature Peaks', fontsize=20)  # y축 글씨 크기 설정
        self.ax_peaks.set_title('Curvature Peaks per Iteration', fontsize=20)  # 제목 글씨 크기 설정
        self.ax_peaks.tick_params(axis='both', labelsize=20)

        self.line_sums, = self.ax_sums.plot([], [], color='navy', label='Curvature Sums', linewidth=5.0)  # 선 굵기 설정
        # self.ax_sums.set_xlabel('Iteration', fontsize=20)  # x축 글씨 크기 설정
        self.ax_sums.set_ylabel('Curvature Sums', fontsize=20)  # y축 글씨 크기 설정
        self.ax_sums.set_title('Curvature Sums per Iteration', fontsize=20)  # 제목 글씨 크기 설정
        self.ax_sums.tick_params(axis='both', labelsize=20)

        self.line_times, = self.ax_times.plot([], [], color='navy', label='Travel Times', linewidth=5.0)  # 선 굵기 설정
        self.ax_times.set_xlabel('Iteration', fontsize=20)  # x축 글씨 크기 설정
        self.ax_times.set_ylabel('Travel Time', fontsize=20)  # y축 글씨 크기 설정
        self.ax_times.set_title('Travel Time per Iteration', fontsize=20)  # 제목 글씨 크기 설정
        self.ax_times.tick_params(axis='both', labelsize=20)
        
        plt.tight_layout()
        plt.subplots_adjust(hspace=0.2)

        # Subscribers for each topic
        rospy.Subscriber('move_base/MyGlobalPlanner/curvature', Float64MultiArray, self.callback_curvature)
        rospy.Subscriber('move_base/MyGlobalPlanner/travel_times', Float64MultiArray, self.callback_travel_times)
        rospy.Subscriber('move_base/MyGlobalPlanner/curvature_sums', Float64MultiArray, self.callback_curvature_sums)
        rospy.Subscriber('move_base/MyGlobalPlanner/curvature_peaks', Float64MultiArray, self.callback_curvature_peaks)

        # Lock for thread safety
        self.lock = threading.Lock()
        self.curvatures = []
        self.travel_times = []
        self.curvature_sums = []
        self.curvature_peaks = []

    def callback_curvature(self, data):
        with self.lock:
            self.curvatures = data.data
            
    def callback_travel_times(self, data):
        with self.lock:
            self.travel_times = data.data
            rospy.loginfo("Received travel times: %s", self.travel_times)

    def callback_curvature_sums(self, data):
        with self.lock:
            self.curvature_sums = data.data
            rospy.loginfo("Received curvature sums: %s", self.curvature_sums)

    def callback_curvature_peaks(self, data):
        with self.lock:
            self.curvature_peaks = data.data
            rospy.loginfo("Received curvature peaks: %s", self.curvature_peaks)

    def update_plot(self):
        with self.lock:
            # # Curvatures plot update
            # if len(self.curvatures) > 0:
            #     self.line_curv.set_xdata(range(len(self.curvatures)))
            #     self.line_curv.set_ydata(self.curvatures)

            # Curvature peaks plot update
            if len(self.curvature_peaks) > 0:
                self.line_peaks.set_xdata(range(1, len(self.curvature_peaks) + 1))
                self.line_peaks.set_ydata(self.curvature_peaks)

            # Curvature sums plot update
            if len(self.curvature_sums) > 0:
                self.line_sums.set_xdata(range(1, len(self.curvature_sums) + 1))
                self.line_sums.set_ydata(self.curvature_sums)

            # Travel times plot update
            if len(self.travel_times) > 0:
                self.line_times.set_xdata(range(1, len(self.travel_times) + 1))
                self.line_times.set_ydata(self.travel_times)

            # Adjust plot limits and redraw for each subplot
            for ax in [self.ax_peaks, self.ax_sums, self.ax_times]:
                ax.relim()
                ax.autoscale_view()

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_plot()
            rate.sleep()

if __name__ == '__main__':
    plotter = CurvaturePlotter()
    plotter.run()
