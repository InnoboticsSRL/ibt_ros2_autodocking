# Copyright 2025 hhakim e Innobotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


#!/usr/bin/env python3
# Import necessary libraries for the ROS 2 node to function.
import rclpy
import sys
import os
from rclpy.node import Node
from rclpy.action import ActionClient
# Import the Docking action definition from the interfaces package.
from ibt_ros2_interfaces.action import Docking
# Import the standard message for battery state.
from sensor_msgs.msg import BatteryState

class DockingClient(Node):
    """
    ROS 2 node that acts as a client for the Docking action and monitors battery status.
    It allows sending 'dock' or 'undock' commands and displays real-time battery percentage.
    """
    def __init__(self):
        """
        Constructor for the DockingClient class.
        Initializes the ROS 2 node, the action client, and variables for battery monitoring.
        """
        super().__init__('docking_client') # Initializes the ROS 2 node with the name 'docking_client'.
        # Creates an action client for 'Docking', connecting to the 'docking' action server.
        self._action_client = ActionClient(self, Docking, 'docking')
        # Initializes variables for the battery subscription and the latest battery message.
        self.battery_subscription = None
        self.latest_battery_msg = None
        # Initializes the timer for periodic battery status printing.
        self.print_timer = None

    # --- Logic for sending an action (dock/undock) ---
    def send_goal(self, command):
        """
        Sends a goal to the docking action server.
        Waits for the server to be available and then sends the specified command.

        Args:
            command (str): The command to send ('dock' or 'undock').

        Returns:
            bool: True if the goal was sent successfully, False otherwise.
        """
        self.get_logger().info('Waiting for docking server...')
        # Waits for the action server to be available with a timeout.
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Docking server not found. Exiting.')
            return False

        # Creates a new goal message for the Docking action.
        goal_msg = Docking.Goal()
        goal_msg.command = command # Sets the command in the goal message.

        self.get_logger().info(f"Sending goal '{command}' to server...")
        # Sends the goal to the server asynchronously.
        # Attaches the feedback callback and the goal response callback.
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def feedback_callback(self, feedback_msg):
        """
        Callback triggered when the action server sends a feedback message.
        Displays the current status provided by the feedback.

        Args:
            feedback_msg (Docking.Feedback): The feedback message received from the server.
        """
        self.get_logger().info(f'FEEDBACK: {feedback_msg.feedback.current_status}')

    def goal_response_callback(self, future):
        """
        Callback triggered when the action server responds to the goal submission (acceptance/rejection).

        Args:
            future: A Future object containing the goal handle.
        """
        goal_handle = future.result()
        # Checks if the goal was accepted by the server.
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            rclpy.shutdown() # Shuts down the ROS 2 runtime if the goal is rejected.
            return

        self.get_logger().info('Goal accepted :)')
        # Gets the final action result asynchronously.
        # Attaches the result callback.
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback triggered when the action server provides the final result of the action.

        Args:
            future: A Future object containing the action result.
        """
        result = future.result().result
        # Displays whether the action was successful and any associated message.
        self.get_logger().info(f'RESULT: Success={result.success}, Message="{result.message}"')
        rclpy.shutdown() # Shuts down the ROS 2 runtime once the result is obtained.

    # --- Battery monitoring logic ---
    def start_percentage_monitoring(self):
        """
        Activates real-time battery monitoring mode.
        Creates a subscription to the /battery_state topic and a timer to print the data.
        """
        self.battery_subscription = self.create_subscription(
            BatteryState,       # Type of message to subscribe to.
            '/battery_state',   # Name of the topic.
            self.battery_callback, # Callback function for received messages.
            10)                 # Message queue size.
        # Creates a timer that will call print_battery_table every 1.0 second.
        self.print_timer = self.create_timer(1.0, self.print_battery_table)

    def battery_callback(self, msg):
        """
        Callback triggered every time a message is received on /battery_state.
        Saves only the latest battery message received.

        Args:
            msg (sensor_msgs.msg.BatteryState): The battery state message.
        """
        self.latest_battery_msg = msg

    def print_battery_table(self):
        """
        Clears the console and prints the battery status formatted in a table.
        This method is called periodically by the timer.
        """
        os.system('clear') # Clears the console screen (works on Unix-like systems).

        # Definitions for table formatting.
        header = "|   BATTERY STATUS   |   VALUE    |"
        separator = "+--------------------+------------+"

        print("--- Real-time Battery Monitor ---")
        print(separator)
        print(header)
        print(separator)

        if self.latest_battery_msg is None:
            # Waiting message if no battery data has arrived yet.
            print("| Waiting for data...  |    ---     |")
        else:
            msg = self.latest_battery_msg
            percentage = msg.percentage * 100 # Converts percentage from [0,1] to [0,100].
            current = msg.current

            # --- Logic to determine charging/discharging status based on current ---
            CHARGE_THRESHOLD = 0.5    # Current threshold to consider battery charging (Amperes).
            DISCHARGE_THRESHOLD = -0.5 # Current threshold to consider battery discharging (Amperes).
            status_str = "NOT CHARGING"  # Default status.
            if current > CHARGE_THRESHOLD:
                status_str = "CHARGING"
            elif current < DISCHARGE_THRESHOLD:
                status_str = "DISCHARGING"
            # --- END LOGIC ---

            # Prints battery data in the table.
            print(f"| Percentage         | {percentage: >8.1f} % |")
            print(f"| Voltage            | {msg.voltage: >8.2f} V |")
            print(f"| Current            | {msg.current: >8.2f} A |")
            print(f"| Status             | {status_str: >10} |")
        
        print(separator)
        print("\n(Press Ctrl+C to exit)") # Instructions for the user to exit.

def main(args=None):
    """
    Main function that initializes the ROS 2 runtime, creates the DockingClient node,
    and manages execution based on command-line arguments.
    """
    rclpy.init(args=args) # Initializes the ROS 2 context.
    client = DockingClient() # Creates an instance of the client node.

    # Checks the validity of command-line arguments.
    if len(sys.argv) < 2 or sys.argv[1] not in ['dock', 'undock', 'percentage']:
        client.get_logger().error('--- ERROR ---')
        client.get_logger().error('Invalid command. Use one of the following:')
        client.get_logger().error('  dock          -> Initiates the docking procedure')
        client.get_logger().error('  undock        -> Initiates the undocking procedure')
        client.get_logger().error('  percentage    -> Displays real-time battery status')
        client.get_logger().error('----------------')
        return # Terminates execution if the command is invalid.

    command = sys.argv[1] # Gets the command from the first argument.
    should_spin = True # Flag to control whether the node should remain spinning.

    if command in ['dock', 'undock']:
        # If the command is 'dock' or 'undock', attempts to send the goal.
        should_spin = client.send_goal(command)
    elif command == 'percentage':
        # If the command is 'percentage', starts battery monitoring.
        client.start_percentage_monitoring()

    if should_spin:
        try:
            # Keeps the node active, processing callbacks and messages.
            rclpy.spin(client)
        except KeyboardInterrupt:
            # Catches keyboard interruption (Ctrl+C) for a clean shutdown.
            print("\nExit requested.")
        finally:
            # Ensures the node is destroyed and the ROS 2 runtime is shut down.
            client.destroy_node() # Releases node resources.
            if rclpy.ok():
                rclpy.shutdown() # Shuts down the ROS 2 runtime.

if __name__ == '__main__':
    # Main entry point of the script.
    main()