#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from std_srvs.srv import Empty

PI = 3.1415926535897


def rotate(speed, angle, clockwise):
    # Starts a new node
    rospy.init_node('AntonCode', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiving the user's input
    print("Let's rotate the Turtle")

    # Converting from angles to radians
    angular_speed = speed * 2 * PI / 360
    relative_angle = angle * 2 * PI / 360

    # We wont use linear components
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while current_angle < relative_angle:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    # Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


def move(speed, distance, is_forward):
    # Starts a new node
    rospy.init_node('AntonCode', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    print("Let's move your robot")

    # Checking if the movement is forward or backwards
    if is_forward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    # Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    # Loop to move the turtle in an specified distance
    while current_distance < distance:
        # Publish the velocity
        velocity_publisher.publish(vel_msg)
        # Takes actual time to velocity calculus
        t1 = rospy.Time.now().to_sec()
        # Calculates distancePoseStamped
        current_distance = speed*(t1-t0)
    # After the loop, stops the robot
    vel_msg.linear.x = 0
    # Force the robot to stop
    velocity_publisher.publish(vel_msg)


def transition():

    pen(1, 2)
    # transition
    rotate(50, 90, 1)
    move(1, 1, 1)
    rotate(50, 90, 0)
    move(0.7, 0.7, 1)
    rotate(50, 90, 0)
    move(3, 3, 1)
    pen(0, 2)


def two():

    # Two
    move(2, 1, 1)
    rotate(90, 90, 1)
    move(2, 1, 1)
    rotate(90, 90, 1)
    move(2, 1, 1)
    rotate(90, 90, 0)
    move(2, 1, 1)
    rotate(90, 90, 0)
    move(2, 1, 1)


def four():

    # Four
    rotate(100, 180, 1)
    move(2, 1, 1)
    rotate(90, 90, 0)
    move(2, 1, 1)
    rotate(90, 90, 0)
    move(2, 1, 1)
    move(2, 2, 0)
    rotate(90, 90, 1)


def three():

    # Three
    rotate(90, 90, 1)
    move(1, 1, 1)
    rotate(100, 135, 1)
    move(1, 1.414, 1)
    rotate(90, 135, 0)
    move(1, 1, 1)
    rotate(90, 135, 1)
    move(1, 1.414, 1)
    rotate(90, 135, 0)

    pen(1, 2)
    move(1, 1, 1)
    pen(0, 2)


def nine():

    # Nine
    rotate(90, 90, 1)
    move(1, 1, 1)
    rotate(90, 90, 1)
    move(2, 1, 1)
    rotate(90, 90, 1)
    move(2, 1, 1)
    rotate(90, 90, 1)
    move(2, 1, 1)
    rotate(90, 90, 1)
    move(2, 1, 1)
    rotate(90, 90, 1)
    move(2, 2, 1)
    rotate(90, 90, 1)
    move(2, 1, 1)
    rotate(90, 180, 1)
    move(2, 1, 1)


def zero():

    # Zero
    rotate(90, 90, 1)
    move(2, 1, 1)
    rotate(90, 90, 1)
    move(2, 2, 1)
    rotate(90, 90, 1)
    move(2, 1, 1)
    rotate(90, 90, 1)
    move(2, 2, 1)
    move(2, 2, 0)
    rotate(90, 90, 1)
    move(2, 1, 1)


def pen(status, width):
    rospy.init_node('AntonCode', anonymous=True)
    rospy.wait_for_service('turtle1/set_pen')
    set_pen = rospy.ServiceProxy('turtle1/set_pen', SetPen)
    set_pen(255, 255, 255, width, status)


def clear():
    rospy.init_node('AntonCode', anonymous=True)
    rospy.wait_for_service('clear')
    fill_bg = rospy.ServiceProxy('clear', Empty)
    rospy.set_param('/background_b', 0)
    rospy.set_param('/background_r', 0)
    rospy.set_param('/background_g', 0)
    fill_bg()


if __name__ == '__main__':
    try:

        clear()
        # Start position
        pen(1, 2)
        move(5, 6.5, 0)
        pen(0, 2)

        two()
        transition()
        four()
        transition()
        three()
        transition()
        nine()
        transition()
        zero()
        transition()
        nine()

    except rospy.ROSInterruptException as ex:
        print(ex)


