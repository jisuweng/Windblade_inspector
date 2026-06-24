#!/usr/bin/env python3
import argparse
import sys

import rospy
from actionlib_msgs.msg import GoalID
from gazebo_msgs.srv import (
    GetJointProperties,
    GetLinkState,
    SetJointProperties,
    SetLinkState,
    SetModelConfiguration,
)
from gazebo_msgs.msg import ODEJointProperties
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def controller_topic(controller, suffix):
    return controller.rstrip("/") + suffix


def read_joint_position_from_joint_states(topic, joint_name, timeout):
    msg = rospy.wait_for_message(topic, JointState, timeout=timeout)
    try:
        index = msg.name.index(joint_name)
    except ValueError:
        return None

    if index >= len(msg.position):
        return None
    return msg.position[index]


def read_joint_position_from_gazebo(model_name, joint_name, timeout):
    service_name = "/gazebo/get_joint_properties"
    rospy.wait_for_service(service_name, timeout=timeout)
    get_joint_properties = rospy.ServiceProxy(service_name, GetJointProperties)

    candidates = [joint_name, "{}::{}".format(model_name, joint_name)]
    for candidate in candidates:
        response = get_joint_properties(candidate)
        if response.success and response.position:
            return response.position[0]
    return None


def get_current_joint_position(args):
    try:
        position = read_joint_position_from_gazebo(args.model, args.joint, args.timeout)
        if position is not None:
            return position
    except (rospy.ROSException, rospy.ServiceException):
        pass

    try:
        position = read_joint_position_from_joint_states(
            args.joint_states_topic, args.joint, args.timeout
        )
        if position is not None:
            return position
    except rospy.ROSException:
        pass

    raise RuntimeError(
        "Could not read current position for joint '{}'. Is the simulation running?".format(
            args.joint
        )
    )


def cancel_controller_goals(controller, settle_time):
    cancel_topic = controller_topic(controller, "/follow_joint_trajectory/cancel")
    publisher = rospy.Publisher(cancel_topic, GoalID, queue_size=1, latch=True)
    rospy.sleep(settle_time)
    publisher.publish(GoalID())


def publish_hold_trajectory(args, position):
    command_topic = controller_topic(args.controller, "/command")
    publisher = rospy.Publisher(command_topic, JointTrajectory, queue_size=1, latch=True)
    rospy.sleep(args.publisher_settle_time)

    trajectory = JointTrajectory()
    trajectory.joint_names = [args.joint]
    trajectory.points = [
        JointTrajectoryPoint(
            positions=[position],
            velocities=[0.0],
            accelerations=[0.0],
            time_from_start=rospy.Duration(args.time_from_start),
        )
    ]

    for _ in range(args.repeat):
        trajectory.header.stamp = rospy.Time(0)
        publisher.publish(trajectory)
        rospy.sleep(args.repeat_interval)


def zero_rotor_link_twist(args):
    if args.no_hard_gazebo:
        return False

    get_service = "/gazebo/get_link_state"
    set_service = "/gazebo/set_link_state"
    rospy.wait_for_service(get_service, timeout=args.timeout)
    rospy.wait_for_service(set_service, timeout=args.timeout)

    get_link_state = rospy.ServiceProxy(get_service, GetLinkState)
    set_link_state = rospy.ServiceProxy(set_service, SetLinkState)

    response = get_link_state(args.rotor_link, "world")
    if not response.success:
        rospy.logwarn("Failed to read rotor link state: %s", response.status_message)
        return False

    state = response.link_state
    state.reference_frame = "world"
    state.twist.linear.x = 0.0
    state.twist.linear.y = 0.0
    state.twist.linear.z = 0.0
    state.twist.angular.x = 0.0
    state.twist.angular.y = 0.0
    state.twist.angular.z = 0.0

    result = set_link_state(state)
    if not result.success:
        rospy.logwarn("Failed to zero rotor link twist: %s", result.status_message)
        return False
    return True


def set_model_joint_position(args, position):
    if args.no_hard_gazebo:
        return False

    service_name = "/gazebo/set_model_configuration"
    rospy.wait_for_service(service_name, timeout=args.timeout)
    set_model_configuration = rospy.ServiceProxy(service_name, SetModelConfiguration)
    response = set_model_configuration(args.model, "", [args.joint], [position])
    if not response.success:
        rospy.logwarn("Failed to set joint position: %s", response.status_message)
        return False
    return True


def set_joint_brake(args, lower, upper, damping, fmax):
    if args.no_hard_gazebo:
        return False

    service_name = "/gazebo/set_joint_properties"
    rospy.wait_for_service(service_name, timeout=args.timeout)
    set_joint_properties = rospy.ServiceProxy(service_name, SetJointProperties)

    config = ODEJointProperties()
    config.damping = [damping]
    config.hiStop = [upper]
    config.loStop = [lower]
    config.erp = [args.brake_erp]
    config.cfm = [0.0]
    config.stop_erp = [args.brake_erp]
    config.stop_cfm = [0.0]
    config.fudge_factor = [0.0]
    config.fmax = [fmax]
    config.vel = [0.0]

    for joint_name in (args.joint, "{}::{}".format(args.model, args.joint)):
        response = set_joint_properties(joint_name, config)
        if response.success:
            return True

    rospy.logwarn("Failed to set brake properties on joint %s", args.joint)
    return False


def lock_joint_at_position(args, position):
    return set_joint_brake(
        args,
        position,
        position,
        args.brake_damping,
        args.brake_fmax,
    )


def release_joint_brake(args):
    return set_joint_brake(
        args,
        args.release_lower,
        args.release_upper,
        args.release_damping,
        args.release_fmax,
    )


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Instantly stop the wind generator blade joint and hold it in place."
    )
    parser.add_argument("--controller", default="/fans_controller")
    parser.add_argument("--joint", default="fans_joint")
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--model", default="wind_generator")
    parser.add_argument("--rotor-link", default="wind_generator::wind_generator_rotor_link")
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--time-from-start", type=float, default=0.02)
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--repeat-interval", type=float, default=0.02)
    parser.add_argument("--publisher-settle-time", type=float, default=0.1)
    parser.add_argument("--brake-duration", type=float, default=0.25)
    parser.add_argument("--brake-damping", type=float, default=100000.0)
    parser.add_argument("--brake-fmax", type=float, default=1000000.0)
    parser.add_argument("--brake-erp", type=float, default=0.8)
    parser.add_argument("--release-lower", type=float, default=-10000.0)
    parser.add_argument("--release-upper", type=float, default=10000.0)
    parser.add_argument("--release-damping", type=float, default=0.0)
    parser.add_argument("--release-fmax", type=float, default=0.0)
    parser.add_argument(
        "--keep-brake",
        action="store_true",
        help="Keep the Gazebo joint limits locked at the stop angle. This is the default.",
    )
    parser.add_argument(
        "--release-brake",
        action="store_true",
        help="Release the hard brake after the stop command is sent.",
    )
    parser.add_argument(
        "--release-only",
        action="store_true",
        help="Release the Gazebo joint brake and exit.",
    )
    parser.add_argument(
        "--no-hard-gazebo",
        action="store_true",
        help="Only command the ROS controller; do not zero the Gazebo rotor link twist.",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rospy.init_node("stop_wind_generator_blades", anonymous=True)

    if args.release_only:
        released = release_joint_brake(args)
        rospy.loginfo("Joint brake release requested: released=%s", released)
        return

    cancel_controller_goals(args.controller, args.publisher_settle_time)
    zeroed_before = zero_rotor_link_twist(args)
    rospy.sleep(args.repeat_interval)

    position = get_current_joint_position(args)
    rospy.loginfo("Stopping %s at %.6f rad", args.joint, position)

    brake_locked = lock_joint_at_position(args, position)
    set_model_joint_position(args, position)
    zero_rotor_link_twist(args)
    publish_hold_trajectory(args, position)
    rospy.sleep(args.brake_duration)
    zeroed_after = zero_rotor_link_twist(args)
    brake_released = False
    if brake_locked and args.release_brake and not args.keep_brake:
        release_joint_brake(args)
        brake_released = True
        publish_hold_trajectory(args, position)

    rospy.loginfo(
        "Blade stop command sent: hold %.6f rad, hard_gazebo=%s, brake_locked=%s",
        position,
        zeroed_before or zeroed_after,
        brake_locked and not brake_released,
    )


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, rospy.ROSException, rospy.ServiceException) as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
