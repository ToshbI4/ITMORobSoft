#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Bot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('antpot', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/base_scan',
                                                LaserScan, self.update_pose)

        self.pose = LaserScan()
        for i in range(179):
            self.pose.ranges.append(1.1)
        self.rate = rospy.Rate(10)

        self.orient = 90
        self.distance_tolerance = 1

    def update_pose(self, data):
        """Callback function which is called when a new message of type LaserScan is
        received by the subscriber."""

        self.pose = data
        print(self.pose.ranges[self.orient])

    def linear_vel(self, constant=0.3):

        return constant * self.pose.ranges[90]

    def start_move(self):
        # distance_tolerance = 1
        vel_msg = Twist()
        while not rospy.is_shutdown():
            while self.pose.ranges[90] >= self.distance_tolerance:
                print("forward")
                # Linear velocity in the x-axis.
                vel_msg.linear.x = self.linear_vel()
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

                # Publish at the desired rate.
                self.rate.sleep()

            # Stopping our robot after the movement is over.
            vel_msg.linear.x = 0
            self.velocity_publisher.publish(vel_msg)

            min_dis = self.pose.ranges[90]

            if self.pose.ranges[0] < self.pose.ranges[179]:
                self.orient = 0
            else:
                self.orient = 179

            self.rotate(min_dis)
            break

        return

    def rotate(self, min_dis):

        vel_msg = Twist()
        while self.pose.ranges[self.orient] > min_dis+0.2:
            print("rotate")
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(min_dis)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        if self.pose.ranges[0] < self.pose.ranges[179]:
            self.orient = 0
        else:
            self.orient = 179

        # Stopping our robot after the movement is over.
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def angular_vel(self, tol, constant=0.03):
        if (self.pose.ranges[self.orient] < tol) and (self.orient == 0) or \
                (self.pose.ranges[self.orient] >= tol) and (self.orient == 179):
            return constant * self.pose.ranges[self.orient]
        else:
            return -1 * constant * self.pose.ranges[self.orient]

    def along_the_wall(self):

        while not rospy.is_shutdown():
            print("Move along")
            #distance_tolerance = 1
            vel_msg = Twist()

            vel_msg.linear.x = self.linear_vel()*10
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(self.distance_tolerance)*100000

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

            # Stopping our robot after the movement is over.
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)

        if self.pose.ranges[0] < self.pose.ranges[179]:
            self.orient = 0
        else:
            self.orient = 179

        return


if __name__ == '__main__':
    try:
        x = Bot()
        x.start_move()

        x.along_the_wall()
    except rospy.ROSInterruptException:
        pass
