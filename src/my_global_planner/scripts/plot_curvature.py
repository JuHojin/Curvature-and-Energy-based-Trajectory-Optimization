#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import threading

class CurvaturePlotter:
    def __init__(self):
        rospy.init_node('curvature_plotter', anonymous=True)

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'r-')
        self.ax.set_xlabel('Index')
        self.ax.set_ylabel('Curvature')
        self.ax.set_title('Curvature of the Path')

        rospy.Subscriber('move_base/MyGlobalPlanner/curvature', Float64MultiArray, self.callback_curvature)

        self.lock = threading.Lock()
        self.curvatures = []

    def callback_curvature(self, data):
        with self.lock:
            self.curvatures = data.data

    def update_plot(self):
        with self.lock:
            if len(self.curvatures) > 0:
                self.line.set_xdata(range(len(self.curvatures)))
                self.line.set_ydata(self.curvatures)

                self.ax.relim()
                self.ax.autoscale_view()
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
