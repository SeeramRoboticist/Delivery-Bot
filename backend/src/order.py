#!/usr/bin/env python

# ROS Imports
import rospy
from std_msgs.msg import String

# For Providing Table arguments
import sys

# Registering a node and Creating Publisher

rospy.init_node("order", anonymous=True)
order_publisher = rospy.Publisher("order", String, queue_size=1)

# Code will wait for Subscriber for reliable data trasnfer
if order_publisher.get_num_connections() == 0:
    rospy.logwarn("Waiting for Subscriber: /order")
while order_publisher.get_num_connections() == 0:
    pass

# List Comprehension for converting the elements str -> int for easy use in main code
order_list = [int(i) for i in sys.argv[1:]]

print(order_list)

# Publishing to the topic "order"
order_publisher.publish(str(order_list))
