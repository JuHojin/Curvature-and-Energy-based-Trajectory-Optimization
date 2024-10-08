#!/usr/bin/env python
import rospy
import numpy as np
from scipy.interpolate import splprep, splev
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from my_global_planner.srv import BSplineSmoothing, BSplineSmoothingResponse

class BsplinePostProcessor:
    def __init__(self):
        # Set up the B-spline smoothing service
        self.bspline_service = rospy.Service('/bspline_smoothing_service', BSplineSmoothing, self.handle_bspline_request)

    def handle_bspline_request(self, req):
        msg = req.input_plan
        n = len(msg.poses)
        if n == 0:
            rospy.logwarn("Received empty path for B-spline smoothing.")
            return BSplineSmoothingResponse(Path())

        x = np.zeros(n)
        y = np.zeros(n)

        for i, pose in enumerate(msg.poses):
            x[i] = pose.pose.position.x
            y[i] = pose.pose.position.y

        # B-spline
        # smoothing factor, s>=0; if s = 0 then no smoothing
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
            print(x_smooth[i], y_smooth[i])
            bspline_path.poses.append(pose)

        rospy.loginfo("Generated B-spline smoothed path.")
        return BSplineSmoothingResponse(output_plan=bspline_path)

if __name__ == '__main__':
    rospy.init_node('bspline_service_node', anonymous=True)
    bspline_processor = BsplinePostProcessor()
    rospy.spin()
