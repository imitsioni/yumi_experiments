#!/usr/bin/env python
import rospy
import sys
from yumi_experiments.msg import RunFoldingAction, ApproachControllerAction, ApproachControllerGoal, AdmittanceControllerGoal, AdmittanceControllerAction, RunFoldingFeedback, SensorCalibrationGoal, SensorCalibrationAction
from std_msgs.msg import Bool
from std_srvs.srv import Empty
import numpy as np
import actionlib

class MoveInTrajectory():

    def __init__(self):
        rospy.init_node('move_trajectory')
        # trajectory_points = rospy.get_param("move_traj/trajectory")

        self.pose_seq = []
        self.goal_cnt = 0
        temp = np.arange(0.1, 0.4, 0.001).tolist()
        for z in temp:
            goal = AdmittanceControllerGoal()
            right_pose = [0.5, -0.2, 0.0+z, -0.64, -0.31, -0.3, 0.627]
            goal.use_right = True
            goal.desired_right_pose.pose.position.x = right_pose[0]
            goal.desired_right_pose.pose.position.y = right_pose[1]
            goal.desired_right_pose.pose.position.z = right_pose[2]
            goal.desired_right_pose.pose.orientation.x = right_pose[3]
            goal.desired_right_pose.pose.orientation.y = right_pose[4]
            goal.desired_right_pose.pose.orientation.z = right_pose[5]
            goal.desired_right_pose.pose.orientation.w = right_pose[6]
            self.pose_seq.append(goal)
        print(len(self.pose_seq))

        self.client = actionlib.SimpleActionClient('move_chop', AdmittanceControllerAction)
        rospy.loginfo("Waiting for move_base action server...")
        # goal = server.accept_new_goal()
        # print("mneh")
        self.server = actionlib.SimpleActionServer('move_chop', AdmittanceControllerAction)

        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.move_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")



    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            if self.goal_cnt < len(self.pose_seq):
                next_goal = AdmittanceControllerGoal()
                # next_goal.desired_right_pose.pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return
        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def move_client(self):

        goal = AdmittanceControllerGoal()
        # goal.desired_right_pose.pose.header.stamp = rospy.Time.now()
        goal.desired_right_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()
if __name__ == '__main__':
    print("am root")

    try:
        MoveInTrajectory()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
