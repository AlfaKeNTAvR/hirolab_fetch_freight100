#!/usr/bin/env python
"""Implements Oculus joystick mapping module for Fetch mobile base.

TODO: Add detailed description.

Author(s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.
    2. Ntmitrii Gyrichidi (girikhidi0@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.

"""

import rospy
import math
import numpy as np

from std_msgs.msg import (Float64MultiArray)
from geometry_msgs.msg import (Twist)

from oculus_ros.msg import (
    ControllerButtons,
    ControllerJoystick,
)


class OculusMobileBaseMapping:
    """
    
    """

    def __init__(
        self,
        controller_side='left',
        max_linear_speed_acceleration_ratio=0.5,
        max_rotation_speed_acceleration_ratio=0.05,
        max_linear_acceleration=1.0,  # Meters/second^2.
        max_rotation_acceleration=1000,  # Degrees/second^2.
        reverse_linear_speed_scale=0.5,
    ):
        """
        
        """

        # # Private constants:

        # # Public constants:
        self.CONTROLLER_SIDE = controller_side

        # Max linear velocities and accelerations:
        self.__MIN_LINEAR_ACCELERATION = 0.1
        self.__MAX_LINEAR_ACCELERATION = max_linear_acceleration

        # Max rotation velocities and accelerations:
        self.__MAX_ROTATION_ACCELERATION = math.radians(
            max_rotation_acceleration
        )
        self.__MIN_ROTATION_ACCELERATION = (
            0.2 * self.__MAX_ROTATION_ACCELERATION
        )

        self.__MAX_LINEAR_SPEED_ACCELERATION_RATIO = (
            max_linear_speed_acceleration_ratio
        )
        self.__MAX_LINEAR_SPEED = (
            self.__MAX_LINEAR_SPEED_ACCELERATION_RATIO
            * self.__MAX_LINEAR_ACCELERATION
        )

        self.__MAX_ROTATION_SPEED_ACCELERATION_RATIO = (
            max_rotation_speed_acceleration_ratio
        )

        self.__MAX_ROTATION_SPEED = (
            self.__MAX_ROTATION_SPEED_ACCELERATION_RATIO
            * self.__MAX_ROTATION_ACCELERATION
        )

        self.__REVERSE_LINEAR_SPEED_SCALE = reverse_linear_speed_scale

        print(
            self.__MAX_ROTATION_ACCELERATION,
            self.__MAX_ROTATION_SPEED,
        )

        # # Private variables:
        self.__oculus_joystick = ControllerJoystick()
        self.__oculus_buttons = ControllerButtons()

        # Max velocities and accelerations:
        self.__max_linear_speed = self.__MAX_LINEAR_SPEED
        self.__max_linear_acceleration = self.__MAX_LINEAR_ACCELERATION

        self.__max_rotation_speed = self.__MAX_ROTATION_SPEED
        self.__max_rotation_acceleration = self.__MAX_ROTATION_ACCELERATION

        # Set target velocities.
        self.__target_linear_velocity = 0.0
        self.__target_rotation_velocity = 0.0

        # Calculated accelerations based on the joystick input.
        self.__current_linear_acceleration = self.__MIN_LINEAR_ACCELERATION
        self.__current_rotation_acceleration = self.__MIN_ROTATION_ACCELERATION

        # Calculated velocities based on the acceleration algorithm.
        self.__current_linear_velocity = 0.0
        self.__current_rotation_velocity = 0.0

        self.__previous_time = rospy.get_time()

        self.__joystick_button_state = 0
        self.__control_mode = 'full'

        # # Public variables:

        # # ROS node:

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__mobilebase_twist_velocity = rospy.Publisher(
            '/base_controller/command',
            Twist,
            queue_size=1,
        )
        self.__current_motion_parameters = rospy.Publisher(
            '/fetch/current_motion_parameters',
            Float64MultiArray,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.CONTROLLER_SIDE}/controller_feedback/joystick',
            ControllerJoystick,
            self.__oculus_joystick_callback,
        )
        rospy.Subscriber(
            f'/{self.CONTROLLER_SIDE}/controller_feedback/buttons',
            ControllerButtons,
            self.__oculus_buttons_callback,
        )
        rospy.Subscriber(
            f'/fetch/max_motion_parameters',
            Float64MultiArray,
            self.__max_motion_parameters_callback,
        )

        # # Timer:

    # # Service handlers:

    # # Topic callbacks:
    def __oculus_joystick_callback(self, message):
        """

        """

        self.__oculus_joystick = message

    def __oculus_buttons_callback(self, message):
        """

        """

        self.__oculus_buttons = message

    def __max_motion_parameters_callback(self, message):
        """
        0 - max linear acceleration,
        1 - max rotation acceleration,
        2 - max rotation velocity.
        
        """

        self.__max_linear_acceleration = message.data[0]
        self.__max_linear_speed = (
            self.__max_linear_acceleration
            * self.__MAX_LINEAR_SPEED_ACCELERATION_RATIO
        )
        self.__max_rotation_acceleration = message.data[1]
        self.__max_rotation_speed = (
            self.__max_rotation_acceleration
            * self.__MAX_ROTATION_SPEED_ACCELERATION_RATIO
        )

    ## Timer functions:

    # # Private methods:
    def __set_target_velocities_accelerations(self):
        """
        
        """

        self.__target_linear_velocity = 0
        self.__target_rotation_velocity = 0

        if abs(self.__oculus_joystick.position_y) > 0.01:  # Noisy joystick.
            # Linear velocity.
            self.__target_linear_velocity = np.interp(
                round(self.__oculus_joystick.position_y, 4),
                [-1.0, 1.0],
                [-self.__max_linear_speed, self.__max_linear_speed],
            )

            # Limit backward motion speed.
            if (
                self.__target_linear_velocity <=
                -self.__REVERSE_LINEAR_SPEED_SCALE * self.__max_linear_speed
            ):
                self.__target_linear_velocity = (
                    -self.__REVERSE_LINEAR_SPEED_SCALE * self.__max_linear_speed
                )

        if abs(self.__oculus_joystick.position_x) > 0.01:  # Noisy joystick.
            # Rotation velocity.
            self.__target_rotation_velocity = np.interp(
                round(self.__oculus_joystick.position_x, 4),
                [-1.0, 1.0],
                [self.__max_rotation_speed, -self.__max_rotation_speed],
            )

    def __update_velocity(
        self,
        target_velocity,
        current_velocity,
        acceleration,
    ):
        """
        
        """

        # Get updated time step.
        current_time = rospy.get_time()
        time_step = current_time - self.__previous_time

        self.__previous_time = current_time

        if current_velocity > target_velocity:
            current_velocity = current_velocity - acceleration * time_step

        elif current_velocity < target_velocity:
            current_velocity = current_velocity + acceleration * time_step

        elif abs(current_velocity - target_velocity) <= 0.01:
            current_velocity = target_velocity

        return current_velocity

    def __update_acceleration(
        self,
        current_velocity,
        max_speed,
        max_acceleration,
        min_acceleration,
    ):
        """
        Maps acceleration to velocity: the bigger the velocity, the bigger the 
        acceleration.
        
        """

        if current_velocity > 0:
            current_acceleration = np.interp(
                abs(current_velocity),
                [
                    0.0,
                    max_speed,
                ],
                [
                    min_acceleration,
                    max_acceleration,
                ],
            )

        elif current_velocity < 0:
            current_acceleration = np.interp(
                abs(current_velocity),
                [
                    0.0,
                    max_speed,
                ],
                [
                    min_acceleration,
                    max_acceleration,
                ],
            )

        # Stop:
        else:
            current_acceleration = min_acceleration

        return current_acceleration

    def __joystick_button_state_machine(self):
        """
        
        """

        # State 0: Joystick button was pressed. Rotation only mode.
        if (
            self.__oculus_joystick.button and self.__joystick_button_state == 0
        ):
            self.__control_mode = 'rotation'
            self.__joystick_button_state = 1

        # State 1: Joystick button was released.
        elif (
            not self.__oculus_joystick.button
            and self.__joystick_button_state == 1
        ):
            self.__joystick_button_state = 2

        # State 2: Joystick button was pressed. Normal control mode.
        if (
            self.__oculus_joystick.button and self.__joystick_button_state == 2
        ):
            self.__control_mode = 'full'
            self.__joystick_button_state = 3

        # State 3: Joystick button was released.
        elif (
            not self.__oculus_joystick.button
            and self.__joystick_button_state == 3
        ):
            self.__joystick_button_state = 0

    def __publish_twist_velocities(self):
        """
        
        """

        twist_message = Twist()
        twist_message.linear.x = self.__current_linear_velocity
        twist_message.angular.z = self.__current_rotation_velocity

        self.__joystick_button_state_machine()

        if self.__control_mode == 'rotation':
            twist_message.linear.x = 0.0

        self.__mobilebase_twist_velocity.publish(twist_message)

    def __publish_current_motion_parameters(self):
        """
        
        """

        message = Float64MultiArray()
        message.data = [
            self.__current_linear_acceleration,
            self.__current_rotation_acceleration,
            self.__current_linear_velocity,
            self.__current_rotation_velocity,
        ]

        self.__current_motion_parameters.publish(message)

    # # Public methods:
    def main_loop(self):
        """
        
        """
        self.__set_target_velocities_accelerations()

        # Velocities:
        self.__current_linear_velocity = self.__update_velocity(
            self.__target_linear_velocity,
            self.__current_linear_velocity,
            self.__current_linear_acceleration,
        )
        self.__current_rotation_velocity = self.__update_velocity(
            self.__target_rotation_velocity,
            self.__current_rotation_velocity,
            self.__current_rotation_acceleration,
        )

        # Accelerations:
        self.__current_linear_acceleration = self.__update_acceleration(
            self.__current_linear_velocity,
            self.__max_linear_speed,
            self.__max_linear_acceleration,
            self.__MIN_LINEAR_ACCELERATION,
        )
        self.__current_rotation_acceleration = self.__update_acceleration(
            self.__current_rotation_velocity,
            self.__max_rotation_speed,
            self.__max_rotation_acceleration,
            self.__MIN_ROTATION_ACCELERATION,
        )

        # Publish calculated velocities:
        self.__publish_twist_velocities()

        # Publish acceleration algorithm feedback:
        self.__publish_current_motion_parameters()


def node_shutdown():
    """
    
    """

    print('\nNode is shutting down...\n')

    # TODO: Stop mobile base motion.

    print('\nNode is shut down.\n')


def main():
    """
    
    """

    # # ROS node:
    rospy.init_node('oculus_mobile_base_mapping')
    rospy.on_shutdown(node_shutdown)

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=1000,
    )

    mobile_base_mapping = OculusMobileBaseMapping()

    print('\nOculus-mobile base mapping is ready.\n')
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        mobile_base_mapping.main_loop()
        # node_rate.sleep()


if __name__ == '__main__':
    main()
