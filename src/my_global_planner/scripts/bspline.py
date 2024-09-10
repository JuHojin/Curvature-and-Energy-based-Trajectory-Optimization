#!/usr/bin/env python
import rospy
import numpy as np
import sys
from scipy.interpolate import splprep, splev
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class BsplinePostProcessor:
    def __init__(self):
        self.uniform_plan_sub = rospy.Subscriber("/move_base/MyGlobalPlanner/plan/uniform", Path, self.uniform_plan_callback)
        self.bspline_plan_pub = rospy.Publisher("/move_base/MyGlobalPlanner/plan/bspline", Path, queue_size=1)

    def uniform_plan_callback(self, msg):
        n = len(msg.poses)
        x = np.zeros(n)
        y = np.zeros(n)

        for i, pose in enumerate(msg.poses):
            x[i] = pose.pose.position.x
            y[i] = pose.pose.position.y

        # B-spline
        # smoothing factor, s>=0; if s = 0 then no smoothing
        # tck: knots, coefficients, degree
        tck, _ = splprep([x, y], s=0.15)  
        u_new = np.linspace(0, 1, len(x) * 10)
        x_smooth, y_smooth = splev(u_new, tck)

        bspline_path = Path()
        bspline_path.header = msg.header

        for i in range(len(x_smooth)):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x_smooth[i]
            pose.pose.position.y = y_smooth[i]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            bspline_path.poses.append(pose)

        self.bspline_plan_pub.publish(bspline_path)
        rospy.loginfo("Published B-spline smoothed path.")

        print("<B-spline Information>")
        print(">> Knots (t):")
        sys.stdout.write("[")
        for knot in tck[0]:
            sys.stdout.write("{:.4f}, ".format(knot))
        sys.stdout.write("]")

        print("\n>> Coefficients (c):")
        for coeff_set in tck[1]:
            sys.stdout.write("[")
            for coeff in coeff_set:
                sys.stdout.write("{:.4f}, ".format(coeff))
            sys.stdout.write("]")

        print("\n>> Degree (k): {}\n".format(tck[2]))

if __name__ == '__main__':
    rospy.init_node('bspline', anonymous=True)
    bspline_processor = BsplinePostProcessor()
    rospy.spin()
