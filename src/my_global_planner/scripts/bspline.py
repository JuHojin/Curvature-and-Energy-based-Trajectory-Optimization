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
        # Get smoothing parameters from ROS parameters
        self.smoothing_factor = rospy.get_param('/bspline_smoothing_service/smoothing_factor', 30)
        self.spline_degree = rospy.get_param('/bspline_smoothing_service/spline_degree', 3)
        self.num_splice_points = rospy.get_param('/bspline_smoothing_service/num_splice_points', 10)
        
        msg = req.input_plan
        n = len(msg.poses)
        if n == 0:
            rospy.logwarn("Received empty path for B-spline smoothing.")
            return BSplineSmoothingResponse(Path())

        # print("smoothing_factor: ", self.smoothing_factor)
        # print("spline_degree: ", self.spline_degree)
        # print("num_splice_points: ", self.num_splice_points)
        
        x = np.zeros(n)
        y = np.zeros(n)

        for i, pose in enumerate(msg.poses):
            x[i] = pose.pose.position.x
            y[i] = pose.pose.position.y

        tck, _ = splprep([x, y], s=self.smoothing_factor, k=self.spline_degree)
        u_new = np.linspace(0, 1, len(x) * self.num_splice_points)
        x_smooth, y_smooth = splev(u_new, tck)

        bspline_path = Path()
        bspline_path.header = msg.header
        bspline_path.poses = []
        
        for i in range(len(x_smooth)):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x_smooth[i]
            pose.pose.position.y = y_smooth[i]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            bspline_path.poses.append(pose)

        # rospy.loginfo("Generated B-spline smoothed path.")
        return BSplineSmoothingResponse(output_plan=bspline_path)

if __name__ == '__main__':
    rospy.init_node('bspline_service_node', anonymous=True)
    bspline_processor = BsplinePostProcessor()
    rospy.spin()
