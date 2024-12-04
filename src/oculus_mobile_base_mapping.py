#!/usr/bin/env python
"""Implements Oculus joystick mapping module for Fetch Freight100 mobile base.

Author(s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2024.
    2. Ntmitrii Gyrichidi (girikhidi0@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.

TODO:
    1. Gear shifting. 
    2. X and Y axis direction calibration per user.
    3. E-braking by pulling the joystick in the opposite: pushing the joystick
       in the opposite direction (by some difference) will trigger a breaking
       condition which will cause a quick decelleration. To reset the breaking
       condition release the joystick.

"""

# # Standart libraries:
import rospy
import numpy as np

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (
    Bool,
    String,
)

from geometry_msgs.msg import (Twist)

# # Third party messages and services:
from oculus_ros.msg import (ControllerJoystick)


class OculusMobileBaseMapping:
    """
    
    """

    def __init__(
        self,
        node_name,
        controller_side,
        max_forward_speed,
        max_angular_speed,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name
        self.__CONTROLLER_SIDE = controller_side

        self.__MAX_FORWARD_SPEED = max_forward_speed
        self.__MAX_ANGULAR_SPEED = max_angular_speed

        # # Public CONSTANTS:

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__oculus_joystick = ControllerJoystick()
        self.__target_velocity = Twist()

        self.__joystick_button_state = 0
        self.__control_mode = 'full'

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.__NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {}

        self.__dependency_status['controller_feedback'] = False

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {}

        self.__dependency_status_topics['controller_feedback'] = (
            rospy.Subscriber(
                f'/{self.__CONTROLLER_SIDE}/controller_feedback/is_initialized',
                Bool,
                self.__controller_feedback_callback,
            )
        )

        # # Service provider:
        # rospy.Service(
        #     f'{self.__NODE_NAME}/<service_name1>',
        #     SetBool,
        #     self.__service_name1_handler,
        # )

        # # Service subscriber:
        # self.__service = rospy.ServiceProxy(
        #     '/<service_name2>',
        #     ServiceType2,
        # )

        # # Topic publisher:
        self.__mobilebase_input_velocity = rospy.Publisher(
            '/mobile_base/velocity_control/input_velocity',
            Twist,
            queue_size=1,
        )
        self.__mapping_control_mode = rospy.Publisher(
            f'{self.__NODE_NAME}/control_mode',
            String,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.__CONTROLLER_SIDE}/controller_feedback/joystick',
            ControllerJoystick,
            self.__oculus_joystick_callback,
        )

        # # Timers:

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __controller_feedback_callback(self, message):
        """Monitors /controller_feedback/is_initialized topic.

        """

        self.__dependency_status['controller_feedback'] = message.data

    # # Service handlers:
    # def __service_name1_handler(self, request):
    #     """

    #     """

    #     success = True
    #     message = ''

    #     return success, message

    # # Topic callbacks:
    def __oculus_joystick_callback(self, message: ControllerJoystick):
        """

        """

        self.__oculus_joystick = message

    # # Timer callbacks:

    # # Private methods:
    # NOTE: By default all new class methods should be private.
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes' is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (f'{self.__NODE_NAME}: '
                         f'lost connection to {key}!')
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key} node...'

            rospy.logwarn_throttle(
                15,
                (
                    f'{self.__NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE (optionally): Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.__NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __joystick_button_state_machine(self, button):
        """
        
        """

        # State 0: Joystick button was pressed. Rotation only mode.
        if (self.__joystick_button_state == 0 and button):
            self.__control_mode = 'rotation'
            self.__joystick_button_state = 1

        # State 1: Joystick button was released.
        elif (self.__joystick_button_state == 1 and not button):
            self.__joystick_button_state = 2

        # State 2: Joystick button was pressed. Normal control mode.
        if (self.__joystick_button_state == 2 and button):
            self.__control_mode = 'full'
            self.__joystick_button_state = 3

        # State 3: Joystick button was released.
        elif (self.__joystick_button_state == 3 and not button):
            self.__joystick_button_state = 0

    def __set_target_velocities(self):
        """
        
        """

        target_linear_velocity = 0
        target_angular_velocity = 0

        # Set target linear velocity:
        if abs(self.__oculus_joystick.position_y) > 0.01:  # Noisy joystick.
            # Linear velocity.
            target_linear_velocity = np.interp(
                round(self.__oculus_joystick.position_y, 4),
                [-1.0, 1.0],
                [-self.__MAX_FORWARD_SPEED, self.__MAX_FORWARD_SPEED],
            )

        # Set target angular velocity:
        if abs(self.__oculus_joystick.position_x) > 0.01:  # Noisy joystick.
            # Rotation velocity.
            target_angular_velocity = np.interp(
                round(self.__oculus_joystick.position_x, 4),
                [-1.0, 1.0],
                [self.__MAX_ANGULAR_SPEED, -self.__MAX_ANGULAR_SPEED],
            )

        self.__target_velocity.linear.x = target_linear_velocity
        self.__target_velocity.angular.z = target_angular_velocity

    def __publish_target_velocities(self):
        """
        
        """

        twist_message = Twist()
        twist_message.linear.x = self.__target_velocity.linear.x
        twist_message.angular.z = self.__target_velocity.angular.z

        self.__joystick_button_state_machine(self.__oculus_joystick.button)

        if self.__control_mode == 'rotation':
            twist_message.linear.x = 0.0

        self.__mobilebase_input_velocity.publish(twist_message)

    def __publish_control_mode(self):
        """
        
        """

        string_message = String()
        string_message.data = self.__control_mode

        self.__mapping_control_mode.publish(string_message)

    # # Public methods:
    # NOTE: By default all new class methods should be private.
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            self.__target_velocity.linear.x = 0
            self.__target_velocity.angular.z = 0
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.
        self.__set_target_velocities()

        self.__publish_target_velocities()
        self.__publish_control_mode()

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.__NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        # NOTE: Placing a service call inside of a try-except block here causes
        # the node to stuck.

        rospy.loginfo_once(f'{self.__NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'oculus_mapping',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    controller_side = rospy.get_param(
        param_name=f'{node_name}/controller_side',
        default='left',
    )
    max_forward_speed = rospy.get_param(
        param_name=f'{rospy.get_name()}/max_forward_speed',
        default=0.5,
    )
    max_angular_speed = rospy.get_param(
        param_name=f'{rospy.get_name()}/max_angular_speed',
        default=0.5,
    )

    class_instance = OculusMobileBaseMapping(
        node_name=node_name,
        controller_side=controller_side,
        max_forward_speed=max_forward_speed,
        max_angular_speed=max_angular_speed,
    )

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
