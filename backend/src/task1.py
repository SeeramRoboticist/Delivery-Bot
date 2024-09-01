#!/usr/bin/env python

# ROS Imports
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

# For Changing type from string to Orginal list
import ast

class deliver1:

    def __init__(self) -> None:

        # ROS neccessary publisher and subscriber for Communication
        self.goal_publish = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.Subscriber("order", String, self.order_cb, queue_size=1)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.nav_result_cb, queue_size=1)

        # Pre-Defined Goals
        self.goals = {
                      "home": {"x": -6.25599, "y": -2.3675, "z": 0.70826, "w": 0.7059},
                      "kitchen": {"x": -5.90492, "y": -0.3311, "z": -0.9998, "w": 0.01839},
                      "table1": {"x": -6.6988, "y": -0.7174, "z": -0.0169, "w": 0.9998},
                      "table2": {"x": -6.8371, "y": -1.3301, "z": 0.0037, "w": 0.9999},
                      "table3": {"x": -6.2695, "y": 0.44065, "z": -0.7109, "w": 0.70327},
                      }
        
        self.order_list = [] # To get the Table numbers
        self.nav_result = False # Navigation result flag

    def navigate_with_postion(self, name):

        """
        This Method gets the name of a 
        pre-defined location and navigates 
        to that position.
        """
        self.nav_result = False

        print(f"Navigating to -> -> {name}\n")

        self.movebase_publish(self.goals[name]["x"], self.goals[name]["y"], self.goals[name]["z"],
                                self.goals[name]["w"])
        
        while not self.nav_result: # To hold the process till the nav is finished
            pass
            
        print(f"Reached -> -> {name}\n")

    def movebase_publish(self, goal_x, goal_y, goal_z, goal_w):

        """
        This method is used to Publish the goal location
        to nav stack.
        """

        goalMsg = PoseStamped()
        goalMsg.header.stamp = rospy.Time.now()
        goalMsg.header.frame_id = "map"
        goalMsg.pose.position.x = goal_x
        goalMsg.pose.position.y = goal_y
        goalMsg.pose.orientation.z = goal_z
        goalMsg.pose.orientation.w = goal_w

        self.goal_publish.publish(goalMsg)

    def nav_result_cb(self, nav_result):

        """
        Move base result callback to understand 
        robot goal has been reached.
        """
        if nav_result.status.text == "Goal reached.":
            self.nav_result = True
        

    def order_cb(self, order):

        """
        Order callback to understand the no. of Tables.
        """

        self.order_list = ast.literal_eval(order.data)

        # Here we read the order list and understand there is a order and 
        # complete that order a simple sequence.
        if len(self.order_list) == 1:

            self.navigate_with_postion("kitchen")

            self.navigate_with_postion("table1")

            self.navigate_with_postion("home")


if __name__ == "__main__":
    """
    Registering the node and creating 
    object for class and spining to do operation 'n' times.
    """
    rospy.init_node("task1", anonymous=True)
    obj = deliver1()
    rospy.spin()