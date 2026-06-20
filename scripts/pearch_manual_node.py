#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3, Quaternion
import time

def main():
    rospy.init_node('pose_publisher_node', anonymous=True)
    pub = rospy.Publisher('/red/tracker/input_pose', MultiDOFJointTrajectoryPoint, queue_size=10)

    time.sleep(1)

    # First pose
    point1 = MultiDOFJointTrajectoryPoint()
    transform1 = Transform()
    transform1.translation = Vector3(x=1.0, y=0.0, z=0.0)
    transform1.rotation = Quaternion(x=0, y=0, z=0, w=1.0)
    point1.transforms = [transform1]

    rospy.loginfo("Publishing pose 1")
    pub.publish(point1)

    rospy.loginfo("Sleeping 10 seconds...")
    time.sleep(5)

    # Second pose
    point2 = MultiDOFJointTrajectoryPoint()
    transform2 = Transform()
    transform2.translation = Vector3(x=2.0, y=0.0, z=0.0)
    transform2.rotation = Quaternion(x=0, y=0, z=0, w=1.0)
    point2.transforms = [transform2]

    rospy.loginfo("Publish pose 2")
    pub.publish(point2)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
