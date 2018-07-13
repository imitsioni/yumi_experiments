#!/usr/bin/env python
import rospy
import sys
from yumi_experiments.msg import RunFoldingAction, ApproachControllerAction, ApproachControllerGoal, AdmittanceControllerGoal, AdmittanceControllerAction, RunFoldingFeedback, SensorCalibrationGoal, SensorCalibrationAction
from std_msgs.msg import Bool
from std_srvs.srv import Empty
import actionlib


def monitor_action_goal(action_server, action_client, action_goal, action_name = "current action", time_limit = float("inf")):
    """Send and monitor an action goal to a given action client.

       The monitor will return in case of the client reporting success, preemption or
       abortion, and will also pass through any incoming preemptions to the action server."""

    success = False
    rospy.loginfo("Sending goal to " + action_name)
    action_client.send_goal(action_goal)
    init_time = rospy.Time.now()
    while action_server.is_active():
       if (rospy.Time.now() - init_time).to_sec() > time_limit:
           rospy.logwarn("Timeout of request, preempting but continuing")
           action_client.cancel_goal()
           success = True
           break

       if action_server.is_preempt_requested():
           rospy.logwarn("Preempting " + action_name)
           action_client.cancel_goal()
           finished = action_client.wait_for_result(timeout = rospy.Duration(1.0))

           if not finished:
               rospy.logerr(action_name + " failed to preempt! This should never happen")
               action_server.set_aborted(text = "Aborted due to action " + action_name + " failing to preempt")
           else:
               action_server.set_preempted(text = "Preempted while running " + action_name)

           break

       if action_client.get_state() == actionlib.GoalStatus.ABORTED:
           rospy.logerr(action_name + " aborted!")
           success = False
           action_server.set_aborted(text = action_name + " aborted")
           break

       if action_client.get_state() == actionlib.GoalStatus.PREEMPTED:
           rospy.logerr(action_name = " preempted!")
           success = False
           action_server.set_aborted(text = action_name + " was preempted")
           break

       if action_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
           rospy.loginfo(action_name + " succeeded!")
           success = True
           break

       rospy.sleep(0.1)

    return success

if __name__ == "__main__":
    """Initialize a chopping experiment. It will set the right arm in an initial configuration
       and call the controller."""

    rospy.init_node("initialize_chopping")
    move_action_name = rospy.get_param("/chopping_initialization/move/action_name", "/yumi/move")
    force_torque_reset_services = rospy.get_param("~ft_reset_services", [])

    if rospy.has_param("/chopping_initialization/right_arm/pose"):
        right_pose = rospy.get_param("/chopping_initialization/right_arm/pose")
    else:
        rospy.logerr("Missing arm initial poses in order to initialize experiment")
        sys.exit(0)

    move_client = actionlib.SimpleActionClient(move_action_name, AdmittanceControllerAction)
    rospy.loginfo("Waiting for move action server...")
    move_client.wait_for_server()


    rospy.loginfo("Waiting for action request...")
    stop_msg = Bool()
    server = actionlib.SimpleActionServer("chopping_server", AdmittanceControllerAction)
    server.start()

    rospy.loginfo("I think I'm doing it ")
    goal = AdmittanceControllerGoal()
    right_pose = [0.5, -0.2, 0.4, -0.64, -0.31, -0.3, 0.627]
    goal.use_right = True
    goal.desired_right_pose.pose.position.x = right_pose[0]
    goal.desired_right_pose.pose.position.y = right_pose[1]
    goal.desired_right_pose.pose.position.z = right_pose[2]
    goal.desired_right_pose.pose.orientation.x = right_pose[3]
    goal.desired_right_pose.pose.orientation.y = right_pose[4]
    goal.desired_right_pose.pose.orientation.z = right_pose[5]
    goal.desired_right_pose.pose.orientation.w = right_pose[6]
    move_client.send_goal(goal)
    server.set_succeeded()



    # experiment_server = actionlib.SimpleActionServer("/calibration/initialize", RunFoldingAction)
    # stop_folding_pub = rospy.Publisher("/folding/disable", Bool, queue_size=1)
    # feedback = RunFoldingFeedback()
    # while not rospy.is_shutdown():
    #     stop_msg.data = False
    #     stop_folding_pub.publish(stop_msg)
    #     while not experiment_server.is_new_goal_available() and not rospy.is_shutdown():  # Wait for goal availability
    #         rospy.loginfo_throttle(60, "Initialization server waiting for goal...")
    #         rospy.sleep(0.5)
    #
    #     if rospy.is_shutdown():
    #         break
    #
    #     # goal = experiment_server.accept_new_goal()
    #     rospy.loginfo("Initializing calibration experiment...")
    #
    #     while experiment_server.is_active():
    #         feedback.current_action = "Admittance control"
    #         arms_move_goal = AdmittanceControllerGoal()
    #         arms_move_goal.use_right = True
    #         arms_move_goal.desired_right_pose.pose.position.x = right_pose[0]
    #         arms_move_goal.desired_right_pose.pose.position.y = right_pose[1]
    #         arms_move_goal.desired_right_pose.pose.position.z = right_pose[2]
    #         arms_move_goal.desired_right_pose.pose.orientation.x = right_pose[3]
    #         arms_move_goal.desired_right_pose.pose.orientation.y = right_pose[4]
    #         arms_move_goal.desired_right_pose.pose.orientation.z = right_pose[5]
    #         arms_move_goal.desired_right_pose.pose.orientation.w = right_pose[6]
    #
    #         experiment_server.publish_feedback(feedback)
    #
    #         success = monitor_action_goal(experiment_server, move_client, arms_move_goal, action_name = move_action_name, time_limit = 10.)
    #
    #         if not success:  # Something went wrong
    #             break
#
            # experiment_server.set_succeeded()
