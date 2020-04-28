#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from std_srvs.srv import Empty

PI = 3.1415926535897


def rotate(speed, angle, clockwise):
    """ Функция поворота черепахи вокруг своей оси"""

    # Инициализируем новый нод
    rospy.init_node('AntonCode', anonymous=True)
    """ Объявляем, что наш нод является издателем (publisher), т.е. он будет отправлять данные для поворота черепахи"""
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    print("Let's rotate the Turtle")

    # Переводим градусы в радианы
    angular_speed = speed * 2 * PI / 360
    relative_angle = angle * 2 * PI / 360

    # только поворачиваемся вдоль оси z, поэтому зануляем ненужные компоненты
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Для различных направлений вращения
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    # Устанавливаем текущее время для вычисления угла поворота
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    
    # Поворачиваем, пока не достигнем нужного угла поворота
    while current_angle < relative_angle:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    # Принудительно останавливаем черепаху
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


def move(speed, distance, is_forward):
    """Функция движения черепахи вдоль прямой линии"""

    # Инициализируем новый нод
    rospy.init_node('AntonCode', anonymous=True)
    """ Объявляем, что наш нод является издателем (publisher), т.е. он будет отправлять данные для поворота черепахи"""
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    print("Let's move your robot")

    # Устанавливаем заданное направление движения черепахи
    if is_forward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)

    # Двигаемся по прямой, поэтому остальные координаты зануляем
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    # Устанавливаем текущее время для вычисления дистанции
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    # Черепаха движется, пока не пройдет заданное расстояние
    while current_distance < distance:
        """ Отправляем скорость, с которой должна двигаться черепаха, если она еще не прошла заданный    путь"""
        velocity_publisher.publish(vel_msg)
        # Вычисляем время
        t1 = rospy.Time.now().to_sec()
        # Вычисляем пройденную дистанцию
        current_distance = speed*(t1-t0)
    # Останавливаем черепаху
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)


def transition():
    """ Функция перехода черепахи между различными цифрами"""

    pen(1, 2)
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
    """ Используем сервис установки цвета ручки."""
    rospy.init_node('AntonCode', anonymous=True)
    rospy.wait_for_service('turtle1/set_pen')
    set_pen = rospy.ServiceProxy('turtle1/set_pen', SetPen)
    set_pen(255, 255, 255, width, status)


def clear():
    """ Используем сервис для очистки окна с черепахой заданным цветом. 
    Это нужно, чтобы в любой ситуации ручка не сливалась с экраном и черепаха перемещалась по заданному цвету."""
    rospy.init_node('AntonCode', anonymous=True)
    rospy.wait_for_service('clear')
    fill_bg = rospy.ServiceProxy('clear', Empty)
    rospy.set_param('/background_b', 0)
    rospy.set_param('/background_r', 0)
    rospy.set_param('/background_g', 0)
    fill_bg()


if __name__ == '__main__':
    try:
	# Устанавливаем цвет фона
        clear()
        # Черепаха перемещается из центра окна в стартовую позицию
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


