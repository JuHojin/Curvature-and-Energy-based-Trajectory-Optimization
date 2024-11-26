#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions

def publish_transforms():
    rospy.init_node('wheel_tf_broadcaster', anonymous=True)
    br = tf2_ros.TransformBroadcaster()

    transforms = [
        # car_1_chassis -> car_1_left_front_wheel
        {
            'parent': "car_1_chassis",
            'child': "car_1_left_front_wheel",
            'translation': [0.325, 0.1, 0],  # Replace with correct values
            'rotation': [1.5708, 0, 0]  # RPY in radians
        },
        # car_1_chassis -> car_1_right_front_wheel
        {
            'parent': "car_1_chassis",
            'child': "car_1_right_front_wheel",
            'translation': [0.325, -0.1, 0],  # Replace with correct values
            'rotation': [1.5708, 0, 0]  # RPY in radians
        },
        # car_1_chassis -> car_1_left_rear_wheel
        {
            'parent': "car_1_chassis",
            'child': "car_1_left_rear_wheel",
            'translation': [0, 0.1, 0],  # Replace with correct values
            'rotation': [1.5708, 0, 0]  # RPY in radians
        },
        # car_1_chassis -> car_1_right_rear_wheel
        {
            'parent': "car_1_chassis",
            'child': "car_1_right_rear_wheel",
            'translation': [0, -0.1, 0],  # Replace with correct values
            'rotation': [1.5708, 0, 0]  # RPY in radians
        },
    ]

    rate = rospy.Rate(100.0)

    while not rospy.is_shutdown():
        for transform in transforms:
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = transform['parent']
            t.child_frame_id = transform['child']
            t.transform.translation.x = transform['translation'][0]
            t.transform.translation.y = transform['translation'][1]
            t.transform.translation.z = transform['translation'][2]

            # Convert RPY to quaternion
            q = tf_conversions.transformations.quaternion_from_euler(
                transform['rotation'][0],
                transform['rotation'][1],
                transform['rotation'][2]
            )
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            br.sendTransform(t)

        rate.sleep()


if __name__ == "__main__":
    try:
        publish_transforms()
    except rospy.ROSInterruptException:
        pass
