#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
My Name is Sahasrajit A.,
This is the Implementation Task 2 - Draw Square for #IRCKT 2021.
"""
import time
from math import radians

# Import ROS.
import rospy
# Importing the required msgs.
from geometry_msgs.msg import Twist, Vector3

# Variables for colour printing.
CGREEN2 = '\33[92m'
CYELLOW2 = '\33[93m'
CBLUE2 = '\33[94m'
CVIOLET2 = '\33[95m'
CBLINK = '\33[5m'
CEND = '\33[0m'


class controller:
    """Controller class for the robot.
    """

    def __init__(self):
        """Initializes the parameters for ROS.
        """
        # Creating a velocity publisher which publishes to the the topic '/cmd_vel' topic.
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # This is the Linear velocity value.
        self.lin_speed = 1
        # This is the Angular velocity value.
        self.ang_speed = 2

    def move(self, dist):
        """Function which translates to the desired distance.

        Args:
            dist (float): The desired distance.
        """

        # Creating a Twist message.
        vel = Twist()

        travelled_distance = 0

        # Stop for safety reasons.
        self.stop()

        # Set Linear and Angular velocities.
        vel.linear = Vector3(self.lin_speed, 0, 0)
        vel.angular = Vector3(0, 0, 0)

        # Start SIM time.
        t0 = rospy.Time.now().to_sec()

        # Traverse said distance.
        while not rospy.is_shutdown():
            if travelled_distance < dist:
                # Publishing the message.
                self.pub.publish(vel)
                # Stop SIM time.
                t1 = rospy.Time.now().to_sec()
                # Calculating the travelled distance.
                travelled_distance = self.lin_speed * (t1-t0)
            else:
                break

        # Stop for safety reasons.
        self.stop()

    def rotate(self, angle, clockwise=False):
        """Function which rotates to the desired angle.

        Args:
            angle (float): Desired angle.
            clockwise (bool, optional): To turn clockwise or no. Defaults to False.
        """
        # Creating a Twist message.
        vel = Twist()
        # Converting the angle to radians.
        angle = radians(angle)

        angle_travelled = 0.0
        speed = self.ang_speed

        # Stop for safety reasons.
        self.stop()

        vel.linear = Vector3(0, 0, 0)

        if clockwise:
            speed = -abs(speed)
        else:
            speed = abs(speed)

        vel.angular = Vector3(0, 0, speed)

        # Start SIM time.
        t0 = rospy.Time.now().to_sec()

        # Traverse said angle.
        while angle_travelled < angle:
            # Stop SIM time.
            t1 = rospy.Time.now().to_sec()
            # Calculating the angle travelled.
            angle_travelled = self.ang_speed * (t1-t0)
            # Publishing the message.
            for i in range(5):
                self.pub.publish(vel)
                rospy.sleep(.1)

        # Stop for safety reasons.
        self.stop()

    def circle(self, radius):
        """Draws a circle.

        Args:
            radius (float): Radius of the circle.
        """
        # Creating a Twist message.
        vel = Twist()

        # Stop for safety reasons.
        self.stop()

        # Set Linear and Angular velocities.
        vel.linear = Vector3(self.lin_speed, 0, 0)
        vel.angular = Vector3(0, 0, self.lin_speed/radius)

        # Start SIM time.
        t0 = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            # Publishing the message.
            self.pub.publish(vel)
            # Stop SIM time.
            t1 = rospy.Time.now().to_sec()
            # If time difference is more than or equal to 6s (time for on revolution) stop.
            if (t1-t0) >= 6.0:
                break

        # Stop for safety reasons.
        self.stop()

    def stop(self):
        """Function which stops the robot.
        """
        # Creating a Twist message.
        vel = Twist()

        # Set Linear and Angular velocities.
        vel.linear = Vector3(0, 0, 0)
        vel.angular = Vector3(0, 0, 0)
        # Publishing the message.
        for _ in range(5):
            self.pub.publish(vel)
            rospy.sleep(.1)


def draw_square():
    """Draws Square.
    """
    # Accessing the global variable ctrl.
    global ctrl
    # Drawing a Square.
    for _ in range(4):
        ctrl.move(1)
        ctrl.rotate(90)
        time.sleep(0.1)


def main():
    """Main function.
    """
    # Initializing node.
    rospy.init_node("draw_square")

    global ctrl
    ctrl = controller()

    rospy.loginfo(CYELLOW2 + "Sahasrajit A.'s Shape maker" + CEND)

    rospy.loginfo(CVIOLET2 + "Drawing a Square." + CEND)
    draw_square()
    rospy.loginfo(CGREEN2 + "Square drawn successfully." + CEND)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
