#!/usr/bin/env python

# ROS Imports
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
import time

# For Changing type from string to Orginal list
import ast

class deliver3:

    def __init__(self) -> None:

        # ROS neccessary publisher and subscriber for Communication
        self.goal_publish = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.Subscriber("order", String, self.order_cb, queue_size=1)
        rospy.Subscriber("move_base/result", MoveBaseActionResult, self.nav_result_cb, queue_size=1)
        rospy.Subscriber("confirmation", String, self.confirmation_cb, queue_size=1)


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
        self.wait_for = 5 * 3
        self.current_process = None
        self.confirm_kitchen = None
        self.confirm_table = None



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

    def confirmation_cb(self, confirmation):

        """
        Confirmation Callback for furter action of robot.
        """
        if confirmation.data == "confirm_kitchen":
            self.confirm_kitchen = True

        if confirmation.data == "confirm_table":
            self.confirm_table = True
        
    def order_cb(self, order):

        """
        Order callback to understand the no. of Tables.
        """
        self.order_list = ast.literal_eval(order.data)

    def home(self):

        """
        Home State robot will be in this method,
        untill we get any orders.
        Once we get order it will Move to kitchen.
        """

        self.order_list = []    # Reseting the order list before starting opreation

        while not rospy.is_shutdown():

            if len(self.order_list) > 0:
                self.navigate_with_postion("kitchen")
                self.current_process = "kitchen"
                return
            
    def kitchen(self):

        """
        Kitchen State: Robot will be in kitchen,
        and wait for furter action, 
        if confirmation comes or time expires, 
        robot will return to home position
        """

        wait_time = time.time() + self.wait_for #Wait time of robot
        self.confirm_kitchen = False    #Resetting flag

        while not rospy.is_shutdown():

            if wait_time < time.time():
                self.navigate_with_postion("home")
                self.current_process = "home"
                return
            
            if self.confirm_kitchen == True:
                self.navigate_with_postion("table1")
                self.current_process = "table"
                return

    def table(self): 

        """
        Table State: Robot will be in Table,
        and wait for furter action, 
        if confirmation comes or time expires, 
        robot will return to home position
        """

        wait_time = time.time() + self.wait_for #Wait time of robot
        self.confirm_table = False    #Resetting flag

        while not rospy.is_shutdown():

            if wait_time < time.time():
                self.navigate_with_postion("kitchen")
                time.sleep(self.wait_for)
                self.navigate_with_postion("home")
                self.current_process = "home"
                return
            
            if self.confirm_table == True:
                self.navigate_with_postion("home")
                self.current_process = "home"
                return

    def main_process(self):

        """
        Main process, understands and switches between processes
        for continuous action.
        """

        self.current_process = "home"

        while not rospy.is_shutdown():

            if self.current_process == "home":
                print(f"current_process -> -> {self.current_process}\n")
                self.home()

            elif self.current_process == "kitchen":
                print(f"current_process -> -> {self.current_process}\n")
                self.kitchen()

            elif self.current_process == "table":
                print(f"current_process -> -> {self.current_process}\n")
                self.table()


if __name__ == "__main__":
    """
    Registering the node and creating 
    object for class and spining to do operation 'n' times.
    """
    rospy.init_node("task3", anonymous=True)
    obj = deliver3()
    obj.main_process()