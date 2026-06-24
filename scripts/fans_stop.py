#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction

def main():
    rospy.init_node('trajectory_stopper')

    client = actionlib.SimpleActionClient('fans_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for joint trajectory action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to joint trajectory action server")

    rospy.loginfo("Cancelling all goals...")
    client.cancel_all_goals()
    rospy.loginfo("Fan stopped.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
