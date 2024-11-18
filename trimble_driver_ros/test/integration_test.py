#  Copyright (c) 2024. Trimble Inc.
#  All rights reserved.

import unittest
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import launch_testing.actions

import rclpy
from rclpy import node
from gsof_msgs.msg import NavigationSolution49
import pytest
import time
import numpy as np


@pytest.mark.launch_test
def generate_test_description():
    dut = Node(
        package='trimble_driver_ros',
        executable='gsof_client_node',
        name=LaunchConfiguration('node_name'),
        parameters=[{
            "ip": "0.0.0.0",  # Change this to the IP of the device
            "port": 5017,
            "parent_frame": "ned",
            "child_frame": "ins",
            "publish_gsof_msgs": True,
            "publish_ros_msgs": True,
            "publish_tf": True,
            "time_source": "now"   # now, gps or gps_time_of_week
        }],
        additional_env={'PYTHONUNBUFFERED': '1'},
        output='screen'
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'node_name',
            default_value='gsof_client',
            description='Name of the node'
        ),
        dut,
        launch_testing.actions.ReadyToTest()
    ])
    context = {'dut': dut}

    return ld, context


class GsofSubscriberNode(node.Node):

    def __init__(self):
        super().__init__('gsof_test_subscriber')
        self.subscription = self.create_subscription(
            NavigationSolution49,
            '/gsof_client/gsof/ins_solution_49',
            self.nav_solution_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.solutions_received = []

    def nav_solution_callback(self, msg: NavigationSolution49):
        self.solutions_received.append(msg)

    def compute_stats(self):
        self.solutions_received: list[NavigationSolution49]
        n = len(self.solutions_received)
        header_times = np.zeros(n, dtype='int64')
        gps_times = np.zeros(n, dtype='int64')

        MS_IN_WEEK = 60 * 60 * 24 * 7 * 1000


        for i, msg in enumerate(self.solutions_received):
            msg = self.solutions_received[i]
            header_times[i] = (msg.header.stamp.sec * 1e9) + msg.header.stamp.nanosec
            gps_times[i] = ((msg.gps_time.week * MS_IN_WEEK) +
                            msg.gps_time.time) * 1000000

        line_coefficients = np.polyfit(gps_times, header_times, 1)
        return line_coefficients


class TestSmokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(self):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Setup and run data collect
        self.node = GsofSubscriberNode()
        end_time = time.time() + 10
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def tearDown(self):
        self.node.destroy_node()

    def test_run_short_time(self, proc_output):
        '''
        Test if the clock offset is constant.

        After letting the driver run for some time, tests if the clock offset between GPS time
        and system time is constant. If it was not constant (i.e. dy/dx != 1) then we have an
        issue where messages were either dropped or were buffered instead of being published
        immediately.

        Parameters
        ----------
        proc_output: The stderr/stdout captures from the device under test.
        '''
        # Change this to the IP of the device
        proc_output.assertWaitFor('GSOF client connection started to 0.0.0.0:5017',
                                  timeout=2)

        assert len(self.node.solutions_received) > 0

        delta_time_line_fit = self.node.compute_stats()
        a = delta_time_line_fit[0]
        b = delta_time_line_fit[1]
        assert a == pytest.approx(1, 0.001)

        print(f'Line fit ax + b = 0 where a = {a} and b = {b} where b is {b / 1000000}ms')
