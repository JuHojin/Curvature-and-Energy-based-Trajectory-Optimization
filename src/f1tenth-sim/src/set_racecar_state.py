#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateResponse

car_name = 'car_1'

def set_racecar_state(req):
    x = req.model_state.pose.position.x
    y = req.model_state.pose.position.y

    state_msg = ModelState()
    # state_msg.reference_frame = 'map'
    state_msg.model_name = car_name
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = 0.001
    state_msg.pose.orientation.x = 0.0
    state_msg.pose.orientation.y = 0.0
    state_msg.pose.orientation.z = 0.0
    state_msg.pose.orientation.w = 1.0

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        resp = set_state(state_msg)

        if resp.success:
            # rospy.loginfo("Reset pos. ({}, {})".format(x, y))
            return SetModelStateResponse(success=True, status_message="Success")
        else:
            # rospy.logerr("Failed")
            return SetModelStateResponse(success=False, status_message="Failed")
        
    except rospy.ServiceException as error_msg:
        rospy.logerr(error_msg)
        return SetModelStateResponse(success=False, status_message="Service call failed")

def racecar_reset_state_service():
    rospy.init_node('racecar_reset_service', anonymous=True)

    service = rospy.Service('/reset_racecar_pose', SetModelState, set_racecar_state)
    # rospy.loginfo('Complete resetting the position')

    rospy.spin()

if __name__ == '__main__':
    try:
        racecar_reset_state_service()
    except rospy.ROSInterruptException:
        pass