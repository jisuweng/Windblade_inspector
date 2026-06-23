#!/usr/bin/env python
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance

def feedback_cb(fb):
    if fb.desired.positions:
        rospy.loginfo("des:%.2f rad  act:%.2f rad  err:%.2f rad",
            fb.desired.positions[0], fb.actual.positions[0], fb.error.positions[0])

def done_cb(state, result):
    status = {0:"PENDING", 1:"ACTIVE", 2:"PREEMPTED", 3:"SUCCEEDED",
              4:"ABORTED", 5:"REJECTED", 8:"RECALLED", 9:"LOST"}
    rospy.loginfo("Goal finished: %s", status.get(state, str(state)))

def main():
    rospy.init_node('trajectory_publisher')

    client = actionlib.SimpleActionClient('fans_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for joint trajectory action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to joint trajectory action server")

    trajectory = JointTrajectory()
    trajectory.joint_names = ['fans_joint']

    points = [
        JointTrajectoryPoint(positions=[0.0],   velocities=[0.0], accelerations=[0.5], time_from_start=rospy.Duration(0)),
        JointTrajectoryPoint(positions=[47.12], velocities=[0.8], accelerations=[0.5], time_from_start=rospy.Duration(60)),
    ]
    trajectory.points = points

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    goal.path_tolerance = [JointTolerance(name='fans_joint', position=0.1, velocity=0.1, acceleration=0.1)]
    goal.goal_tolerance = [JointTolerance(name='fans_joint', position=0.1, velocity=0.1, acceleration=0.1)]

    rospy.loginfo("Sending goal, Ctrl+C to stop...")
    client.send_goal(goal, done_cb=done_cb, feedback_cb=feedback_cb)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
