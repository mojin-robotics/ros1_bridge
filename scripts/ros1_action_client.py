#! /usr/bin/env python3

from __future__ import print_function

import sys
import random

import rospy
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from action_tutorials_interfaces.msg import FibonacciAction, FibonacciGoal, FibonacciFeedback


class FibClient:

    def __init__(self):
        # Creates the SimpleActionClient, passing the type of the action
        # (FibonacciAction) to the constructor.
        self._ac = actionlib.SimpleActionClient('fibonacci', FibonacciAction)

        # Waits until the action server has started up and started
        # listening for goals.
        self._ac.wait_for_server()

    def run(self):
        # Creates a goal to send to the action server.
        goal = FibonacciGoal(order=20)

        # Sends the goal to the action server.
        self._ac.send_goal(goal, feedback_cb=self.accept_feedback)

        # Waits for the server to finish performing the action.
        self._ac.wait_for_result()

        # Prints out the result of executing the action
        return self._ac.get_result()  # A FibonacciResult

    def accept_feedback(self, feedback: FibonacciFeedback):
        rospy.loginfo(f"Got feedback: {feedback}")
        # if random.randint(0, 1):
        #     rospy.loginfo("Randomly aborting")
        #     self._ac.cancel_all_goals()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        client = FibClient()
        for i in range(1000):
            result = client.run()
            print(f"Result {i}:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)