# Import necessary libraries
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState # For subscribing to joint state messages

# For publishing joint commands
from interbotix_xs_msgs.msg import (
    JointGroupCommand,
    JointSingleCommand,
    JointTrajectoryCommand
) 

# Service for setting operating modes
from interbotix_xs_msgs.srv import (
    OperatingModes,
) 

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS # Interbotix robot manipulation library
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup # Robot lifecycle management

import time
import math
import socket
import numpy as np

from Gravity_Compensation_Function import calculate_gravity # Import custom gravity compensation function


# Define the controller class for the ViperX300s robotic arm
class ViperX300sController(Node):
    def __init__(self):
        super().__init__('ViperX_300s_Controller') # Initialize the ROS2 node with a name

        # Call the service to set the operating mode
        self.call_service()

        # Create publisher for joint commands
        self.publisher = self.create_publisher(JointGroupCommand, '/vx300s/commands/joint_group', 10)

        # Subscribe to joint states topic
        self.subscription = self.create_subscription(JointState, '/vx300s/joint_states', self.listener_callback, 10)

        # Store the most recent message without processing immediately
        self.recent_joint_state = None

        # Timer to process joint states at a specific rate (200Hz in this case)
        self.timer = self.create_timer(0.005, self.process_joint_state)

        # Initialize variables for PD control and joint currents
        self.prev_error = [0] * 9
        self.joint_currents = [0] * 9
        self.PD_command = [0] * 9


        # Proportional (Kp) and derivative (Kd) gains for PID control, tuned for each joint

        self.kp = {
            'waist': 100,
            'shoulder': 5000,
            'elbow': 4500,
            'forearm_roll': 1500,
            'wrist_angle': 700,
            'wrist_rotate':500,
        }


   
        self.kd = {
            'waist': 5,
            'shoulder': 350,
            'elbow': 200,
            'forearm_roll': 10,
            'wrist_angle': 30,
            'wrist_rotate': 25,
        }        
    
        # Limits for joint currents
        self.u_min = [-3200.0] * 9
        self.u_max = [3200.0] * 9


        # Desired positions for each joint in radians
        self.desired_positions = {
            'waist': 0.0,
            'shoulder': 0.0,
            'elbow': 0.0,
            'forearm_roll': 0.0,
            'wrist_angle': 0.0,
            'wrist_rotate': 0.0,
        }
        # Variable to store the time of the last update
        self.previous_time = time.time()

    def call_service(self):
        # Service client to set the operating modes of the robotic arm
        client = self.create_client(OperatingModes, '/vx300s/set_operating_modes')
        while not client.wait_for_service(timeout_sec=1.0):  # Wait for the service to be available
            self.get_logger().info('Service not available, waiting...')

        # Create a request to set the mode to 'current'    
        request = OperatingModes.Request()
        request.cmd_type='group'
        request.name = 'arm'
        request.mode = 'current'

        # Call the service asynchronously and wait for a response
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Service call successful')
        else:
            self.get_logger().error('Service call failed')

    def listener_callback(self, msg):
        # Store the incoming joint state message for processing later
        self.recent_joint_state = msg

    def process_joint_state(self):
        if self.recent_joint_state is None:
            return  # No data to process yet


        # Extract joint names and positions
        joint_names = self.recent_joint_state.name
        joint_positions = self.recent_joint_state.position

        # Calculate the time elapsed since the last update
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time  # Update previous time

        # PD control logic for joints
        for i, joint_name in enumerate(joint_names[:6]):
            if joint_name in self.desired_positions:
                desired_position = self.desired_positions[joint_name]
                error = desired_position - joint_positions[i]
                p_error = error
                d_error = (error - self.prev_error[i]) / dt

                u = self.kp[joint_name] * p_error + self.kd[joint_name] * d_error


                self.PD_command[i] = u
                self.prev_error[i] = error

        
        
        # Calculate the gravity compensation vector based on the current state configuration
        q1, q2, q3, q4, q5, q6 = joint_positions[:6]  
        G = calculate_gravity(q1, q2, q3, q4, q5, q6)



        # By uncommenting the code lines below, you will have PD+G Tracking control



        # self.joint_currents[0] = max(min(self.PD_command[0] + float(G[0]), 3200.0), -3200.0)
        # self.joint_currents[1] = max(min(self.PD_command[1] + float(G[1]), 3200.0), -3200.0)
        # self.joint_currents[2] = max(min(self.PD_command[2] + float(G[2]), 3200.0), -3200.0)
        # self.joint_currents[3] = max(min(self.PD_command[3] + float(G[3]), 3200.0), -3200.0)
        # self.joint_currents[4] = max(min(self.PD_command[4] + float(G[4]), 3200.0), -3200.0)
        # self.joint_currents[5] = max(min(self.PD_command[5] + float(G[5]), 3200.0), -3200.0)

 
        # The lines below correspond to the gravity compensation only without any setpoints tracking. If tracking control is desired, these lines should be commented after
        # the above line are being uncommented

        self.joint_currents[0]=float(G[0])
        self.joint_currents[1]=float(G[1])
        self.joint_currents[2]=float(G[2])
        self.joint_currents[3]=float(G[3])
        self.joint_currents[4]=float(G[4])
        self.joint_currents[5]=float(G[5])

        # Publish joint currents as commands
        jointcommand = JointGroupCommand()
        jointcommand.name = 'arm'

        jointcommand.cmd = [self.joint_currents[0], self.joint_currents[1], self.joint_currents[2], self.joint_currents[3], self.joint_currents[4], self.joint_currents[5]]
        self.publisher.publish(jointcommand)
        # self.get_logger().info(f'Gravity Compensation: {G[0],G[1], G[2], G[3], G[4], G[5]}')
        


# Entry point of the script
def main(args=None):
    rclpy.init(args=args) # Initialize ROS2
    viperx_300s_controller = ViperX300sController()  # Create the controller object
    rclpy.spin(viperx_300s_controller) # Keep the node running

    # Clean up resources
    viperx_300s_controller.destroy_node()
    rclpy.shutdown()


# Run the script
if __name__ == '__main__':
    main()


