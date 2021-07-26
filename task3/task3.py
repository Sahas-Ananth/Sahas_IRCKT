#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Main.ui'
#
# Created by: PyQt5 UI code generator 5.10.1

"""
My Name is Sahasrajit A.,
This is the Implementation Task 3 - Waiter bot at cafe for #IRCKT 2021.
"""

import sys
# For Multi-Threading.
import threading
import time
from math import atan2, pow, sqrt

# Import ROS.
import rospy
# Importing the required msgs.
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
# For GUI.
from PyQt5 import QtCore, QtGui, QtWidgets
# For Transformations.
from tf.transformations import euler_from_quaternion


# Variables for colour printing.
CRED2 = '\33[91m'
CGREEN2 = '\33[92m'
CYELLOW2 = '\33[93m'
CBLUE2 = '\33[94m'
CVIOLET2 = '\33[95m'
CBLINK = '\33[5m'
CEND = '\33[0m'


class robot_controller:
    """Controller class for the robot and is the front-end for ROS.
    """

    def __init__(self):
        """Initializes the parameters for ROS
        """
        # Initializing the ROS Node.
        rospy.init_node("task3_controller")

        # Printing/Logging My name.
        rospy.loginfo(
            CVIOLET2 + CBLINK + "Sahasrajit A.'s IRCKT 2021 Task 3 Waiter Robot for Cafe" + CEND)

        # Creating a velocity publisher which publishes to the the topic '/cmd_vel' topic.
        self.vel_pub = rospy.Publisher(
            name="cmd_vel", data_class=Twist, queue_size=10)

        # Creating a subscriber for the topic '/odom'.
        rospy.Subscriber(
            name="odom", data_class=Odometry, callback=self.odom_cb)

        # Creating a message of type Twist.
        self.vel_msg = Twist()
        # This is the Emergency Stop Flag. Will be further referred as ES Flag.
        self.es_flag = False
        # This is the angular velocity value (w). This is closest approximation to the greek letter w.
        self.w = 0.5
        # This is the linear velocity value (v).
        self.v = 0.5

    def odom_cb(self, msg):
        """This is the callback function for the Odometry publisher. Takes the Current odometry value and converts it into position of the robot in the x-y plane. Also converts the Orientation in Quaternion (qw, qx, qy, qz) to Euler angles (Roll, Pitch, and Yaw).

        Args:
                msg (nav_msgs/Odometry): Current odometry value from the robot.
        """

        # Getting the x and y.
        self.x, self.y = (msg.pose.pose.position.x,
                          msg.pose.pose.position.y)

        # Getting the quaternions.
        qw, qx, qy, qz = (msg.pose.pose.orientation.w,
                          msg.pose.pose.orientation.x,
                          msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z)

        # Since the roll and pitch isn't needed (2D robot) they are stored in a dummy variable. Yaw is alone stored.
        _, _, self.theta = euler_from_quaternion([qx, qy, qz, qw])

    def go(self, goal_x, goal_y, table):
        """Wrapper function for the run function which creates a sepearate thread for goal processing.

        Args:
            goal_x (float): x-coordinate of the goal.
            goal_y (float): y-coordinate of the goal.
            table (boolean): Is the given goal a table?
        """
        # Create a new thread for goal processing.
        threading.Thread(target=lambda:
                         self.run(goal_x, goal_y, table)).start()

    def run(self, goal_x, goal_y, table):
        """Function responsible for goal achievement.

        Args:
            goal_x (float): x-coordinate of the goal.
            goal_y (float): y-coordinate of the goal.
            table (boolean): Is the given goal a table?
        """

        # Stop for safety.
        self.stop()
        # Rotate to the angle of the goal.
        self.rotate(goal_x, goal_y)
        # Move to the position of the goal.
        self.move(goal_x, goal_y)
        # Stop for safety.
        self.stop()

        # If it's a table and the ES flag isn't True then  sleep for 2 secs and print "Order Delievered".
        if table and not self.es_flag:
            time.sleep(2)
            rospy.loginfo(CGREEN2 + "Order Delievered" + CEND)

        # Rotate to the angle of home (Chef's Kitchen).
        self.rotate(-0.66, 0.64)
        # Move to the position of home (Chef's Kitchen).
        self.move(-0.66, 0.64)
        # Stop for safety.
        self.stop()
        # Log/Print "Ready for Next Order" if the ES flag is not set else "Mission Aborted"
        rospy.loginfo(
            (CGREEN2 + "Ready for Next Order" + CEND) if not self.es_flag else (CRED2 + "Mission Aborted" + CEND))

    def rotate(self, goal_x, goal_y):
        """Function which rotates to the angle facing the goal. 

        Args:
            goal_x (float): x-coordinate of the goal.
            goal_y (float): y-coordinate of the goal.
        """

        # Te rate of the publisher loop. Sets how many times a second the message gets published.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.es_flag:
            # The difference between the goal and current position.
            dx = goal_x - self.x
            dy = goal_y - self.y

            # Desired angle.
            ang = atan2(dy, dx)

            # Achieves the desired angle.
            if round(abs(ang - self.theta), 2) < 0.05:
                self.stop()
                break
            else:
                # Rotating to the nearest side.
                if round(abs(ang - self.theta), 2) > 0:
                    self.vel_msg.angular = Vector3(0, 0, -self.w)
                else:
                    self.vel_msg.angular = Vector3(0, 0, self.w)

            # Publishing the message.
            self.vel_pub.publish(self.vel_msg)
            # Sleeping.
            rate.sleep()

    def move(self, goal_x, goal_y):
        """Function which translates to the goal.

        Args:
            goal_x (float): x-coordinate of the goal.
            goal_y (float): y-coordinate of the goal.
        """

        # The rate of the publisher loop. Sets how many times a second the message gets published.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.es_flag:
            # The difference between the goal and current position.
            dx = goal_x - self.x
            dy = goal_y - self.y

            # Error to the Desired position.
            dmag = sqrt(pow(dx, 2) + pow(dy, 2))

            # Achieves the desired position.
            if round(abs(dmag), 2) < 0.10:
                self.stop()
                break
            else:
                self.vel_msg.linear = Vector3(self.v, 0, 0)

            # Publishing the message.
            self.vel_pub.publish(self.vel_msg)
            # Sleeping.
            rate.sleep()

    def stop(self):
        """Function which stops the robot.
        """
        self.vel_msg.linear = Vector3(0, 0, 0)
        self.vel_msg.angular = Vector3(0, 0, 0)
        for _ in range(5):
            # Publishing the message.
            self.vel_pub.publish(self.vel_msg)
            # Sleeping.
            rospy.sleep(.1)


class gui_handler:
    """Class which handles GUI functions and routines. An interface between ROS and GUI.
    """

    def __init__(self):
        """Initializes the object for translation between ROS and GUI.
        """
        # Object for handling the ROS functions.
        self.robot = robot_controller()
        # Definitions of all the available goals.
        self.goals = {"home": [-0.66, 0.64],
                      "tb1": [-0.35, 0],
                      "tb2": [-0.66, -0.64],
                      "tb3": [0.33, -0.64],
                      "tb4": [0.66, 0.015],
                      "tb5": [0.33, 0.66]
                      }

    def table_pressed(self, number):
        """Function responsible for getting the which table is pressed and sending the robot its goal based on the table selected.

        Args:
            number (int): Table number to which the robot must go.
        """
        # Formatting the table number so it can be printed and retrived from the list.
        g = "tb{}".format(str(number))

        # Logging/Printing the Goal.
        rospy.loginfo(CBLUE2 + "Going to Table {}".format(str(number)) + CEND)

        # Sending the robot the goal coordinates.
        self.robot.go(goal_x=self.goals[g][0],
                      goal_y=self.goals[g][1], table=True)

    def hb_pressed(self):
        """Function responding when Home button is pressed.
        """
        # Caling the ES function as Home =  ES + goal.
        self.es_pressed()

        # Logging/Printing the Goal.
        rospy.loginfo(CBLUE2 + "Going to Chef's Kitchen" + CEND)

        # Sending the robot the goal coordinates.
        self.robot.go(goal_x=self.goals["home"][0],
                      goal_y=self.goals["home"][1], table=False)

    def es_pressed(self):
        """Function responding when Emergency Stop button is pressed.
        """
        self.robot.es_flag = True
        time.sleep(2)
        self.robot.es_flag = False

    def slider_changed(self, name, value):
        """Function responding when either the linear and/or angular velocity changer slider is changed. 

        Args:
            name (string): Which slider is changed (Linear/Angular)?
            value (int): Current value of the slider.
        """
        if name == "linear":
            self.robot.v = 0.5 + (0.1*value)
        else:
            self.robot.w = 0.5 + (0.1*value)

        # Printing the values.
        rospy.loginfo(
            CYELLOW2 + "Linear velocity value: {}m/s, Angular velocity value: {}rad/s".format(self.robot.v, self.robot.w) + CEND)


class Ui_Form(object):
    """Class responsible for generation of GUI.
    """

    def setupUi(self, Form):
        """Sets Up the UI.
        """

        # Auto generated code my PyQT5/pyuic5.
        Form.setObjectName("Form")
        Form.resize(500, 340)
        Form.setMinimumSize(QtCore.QSize(500, 340))
        Form.setMaximumSize(QtCore.QSize(500, 340))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        Form.setFont(font)
        Form.setAutoFillBackground(False)
        self.Waypoints_frame = QtWidgets.QFrame(Form)
        self.Waypoints_frame.setGeometry(QtCore.QRect(10, 50, 241, 280))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.Waypoints_frame.setFont(font)
        self.Waypoints_frame.setAutoFillBackground(False)
        self.Waypoints_frame.setStyleSheet("")
        self.Waypoints_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.Waypoints_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.Waypoints_frame.setObjectName("Waypoints_frame")
        self.emergency_stop_button = QtWidgets.QPushButton(
            self.Waypoints_frame)
        self.emergency_stop_button.setGeometry(QtCore.QRect(10, 208, 220, 50))
        self.emergency_stop_button.setMinimumSize(QtCore.QSize(20, 50))
        self.emergency_stop_button.setMaximumSize(
            QtCore.QSize(16777215, 16777215))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.emergency_stop_button.setFont(font)
        self.emergency_stop_button.setStyleSheet("background-color : red")
        self.emergency_stop_button.setObjectName("emergency_stop_button")
        self.table5_button = QtWidgets.QPushButton(self.Waypoints_frame)
        self.table5_button.setGeometry(QtCore.QRect(10, 144, 100, 50))
        self.table5_button.setMinimumSize(QtCore.QSize(20, 50))
        self.table5_button.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.table5_button.setFont(font)
        self.table5_button.setStyleSheet(
            "background-color: rgb(252, 233, 79);")
        self.table5_button.setObjectName("table5_button")
        self.table2_button = QtWidgets.QPushButton(self.Waypoints_frame)
        self.table2_button.setGeometry(QtCore.QRect(130, 20, 100, 50))
        self.table2_button.setMinimumSize(QtCore.QSize(20, 50))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.table2_button.setFont(font)
        self.table2_button.setStyleSheet(
            "background-color: rgb(252, 233, 79);")
        self.table2_button.setObjectName("table2_button")
        self.table3_button = QtWidgets.QPushButton(self.Waypoints_frame)
        self.table3_button.setGeometry(QtCore.QRect(10, 80, 100, 50))
        self.table3_button.setMinimumSize(QtCore.QSize(20, 50))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.table3_button.setFont(font)
        self.table3_button.setStyleSheet(
            "background-color: rgb(252, 233, 79);")
        self.table3_button.setObjectName("table3_button")
        self.table1_button = QtWidgets.QPushButton(self.Waypoints_frame)
        self.table1_button.setEnabled(True)
        self.table1_button.setGeometry(QtCore.QRect(10, 18, 100, 50))
        self.table1_button.setMinimumSize(QtCore.QSize(20, 50))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.table1_button.setFont(font)
        self.table1_button.setWhatsThis("")
        self.table1_button.setStyleSheet(
            "background-color: rgb(252, 233, 79);")
        self.table1_button.setObjectName("table1_button")
        self.table4_button = QtWidgets.QPushButton(self.Waypoints_frame)
        self.table4_button.setGeometry(QtCore.QRect(130, 80, 100, 50))
        self.table4_button.setMinimumSize(QtCore.QSize(20, 50))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.table4_button.setFont(font)
        self.table4_button.setStyleSheet(
            "background-color: rgb(252, 233, 79);")
        self.table4_button.setObjectName("table4_button")
        self.home_button = QtWidgets.QPushButton(self.Waypoints_frame)
        self.home_button.setGeometry(QtCore.QRect(130, 144, 100, 50))
        self.home_button.setMinimumSize(QtCore.QSize(20, 50))
        self.home_button.setMaximumSize(QtCore.QSize(100, 16777215))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.home_button.setFont(font)
        self.home_button.setStyleSheet("background-color: rgb(50, 230, 50);")
        self.home_button.setObjectName("home_button")
        self.label_2 = QtWidgets.QLabel(Form)
        self.label_2.setGeometry(QtCore.QRect(300, 10, 161, 31))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        font.setPointSize(16)
        self.label_2.setFont(font)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.label = QtWidgets.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(80, 10, 111, 31))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        font.setPointSize(16)
        self.label.setFont(font)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.VelocityControl_frame = QtWidgets.QFrame(Form)
        self.VelocityControl_frame.setGeometry(QtCore.QRect(260, 50, 230, 280))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.VelocityControl_frame.setFont(font)
        self.VelocityControl_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.VelocityControl_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.VelocityControl_frame.setObjectName("VelocityControl_frame")
        self.av_slider = QtWidgets.QSlider(self.VelocityControl_frame)
        self.av_slider.setGeometry(QtCore.QRect(150, 10, 60, 240))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.av_slider.setFont(font)
        self.av_slider.setMaximum(10)
        self.av_slider.setPageStep(5)
        self.av_slider.setOrientation(QtCore.Qt.Vertical)
        self.av_slider.setObjectName("av_slider")
        self.lv_slider = QtWidgets.QSlider(self.VelocityControl_frame)
        self.lv_slider.setGeometry(QtCore.QRect(30, 10, 60, 240))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.lv_slider.setFont(font)
        self.lv_slider.setMaximum(10)
        self.lv_slider.setPageStep(5)
        self.lv_slider.setOrientation(QtCore.Qt.Vertical)
        self.lv_slider.setTickPosition(QtWidgets.QSlider.NoTicks)
        self.lv_slider.setTickInterval(10)
        self.lv_slider.setObjectName("lv_slider")
        self.label_3 = QtWidgets.QLabel(self.VelocityControl_frame)
        self.label_3.setGeometry(QtCore.QRect(20, 250, 67, 31))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        font.setPointSize(14)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.line = QtWidgets.QFrame(self.VelocityControl_frame)
        self.line.setGeometry(QtCore.QRect(100, 0, 41, 281))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.line.setFont(font)
        self.line.setFrameShape(QtWidgets.QFrame.VLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.label_4 = QtWidgets.QLabel(self.VelocityControl_frame)
        self.label_4.setGeometry(QtCore.QRect(150, 250, 67, 31))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        font.setPointSize(14)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.VelocityControl_frame)
        self.label_5.setGeometry(QtCore.QRect(70, 230, 20, 20))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(self.VelocityControl_frame)
        self.label_6.setGeometry(QtCore.QRect(190, 230, 20, 20))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.label_6.setFont(font)
        self.label_6.setObjectName("label_6")
        self.label_7 = QtWidgets.QLabel(self.VelocityControl_frame)
        self.label_7.setGeometry(QtCore.QRect(70, 5, 20, 20))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.label_7.setFont(font)
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(self.VelocityControl_frame)
        self.label_8.setGeometry(QtCore.QRect(190, 5, 20, 20))
        font = QtGui.QFont()
        font.setFamily("Open Sans")
        self.label_8.setFont(font)
        self.label_8.setObjectName("label_8")
        self.label.raise_()
        self.Waypoints_frame.raise_()
        self.label_2.raise_()
        self.VelocityControl_frame.raise_()

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

        # GUI Handling and adding functionalities to buttons and slider.

        self.gh = gui_handler()
        self.table1_button.pressed.connect(lambda: self.gh.table_pressed(1))
        self.table2_button.pressed.connect(lambda: self.gh.table_pressed(2))
        self.table3_button.pressed.connect(lambda: self.gh.table_pressed(3))
        self.table4_button.pressed.connect(lambda: self.gh.table_pressed(4))
        self.table5_button.pressed.connect(lambda: self.gh.table_pressed(5))

        self.home_button.pressed.connect(self.gh.hb_pressed)
        self.emergency_stop_button.pressed.connect(self.gh.es_pressed)

        self.lv_slider.valueChanged.connect(
            lambda: self.gh.slider_changed("linear", self.lv_slider.value()))
        self.av_slider.valueChanged.connect(
            lambda: self.gh.slider_changed("angular", self.av_slider.value()))

    def retranslateUi(self, Form):
        """Retranslates UI. Autogenerated code by PyQt5/pyuic5.
        """
        # Auto generated code my PyQT5/pyuic5.

        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Sahas\' RBLbot Control"))
        self.emergency_stop_button.setToolTip(
            _translate("Form", "Emergency Stop Button"))
        self.emergency_stop_button.setText(_translate("Form", "EMERGENY STOP"))
        self.table5_button.setToolTip(_translate("Form", "Table 5 Button"))
        self.table5_button.setText(_translate("Form", "Table 5"))
        self.table2_button.setToolTip(_translate("Form", "Table 2 Button"))
        self.table2_button.setText(_translate("Form", "Table 2"))
        self.table3_button.setToolTip(_translate("Form", "Table 3 Button"))
        self.table3_button.setText(_translate("Form", "Table 3"))
        self.table1_button.setToolTip(_translate("Form", "Table 1 Button"))
        self.table1_button.setText(_translate("Form", "Table 1"))
        self.table4_button.setToolTip(_translate("Form", "Table 4 Button"))
        self.table4_button.setText(_translate("Form", "Table 4"))
        self.home_button.setToolTip(
            _translate("Form", "Return to Home Button"))
        self.home_button.setText(_translate("Form", "HOME"))
        self.label_2.setText(_translate("Form", "Velocity Control"))
        self.label.setText(_translate("Form", "Waypoints"))
        self.av_slider.setToolTip(_translate(
            "Form", "Slider for control of Angular Velocity"))
        self.lv_slider.setToolTip(_translate(
            "Form", "Slider for control of Linear Velocity"))
        self.label_3.setText(_translate("Form", "Linear"))
        self.label_4.setText(_translate("Form", "Angular"))
        self.label_5.setText(_translate("Form", "0"))
        self.label_6.setText(_translate("Form", "0"))
        self.label_7.setText(_translate("Form", "10"))
        self.label_8.setText(_translate("Form", "10"))


if __name__ == "__main__":
    # Driver code.
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
