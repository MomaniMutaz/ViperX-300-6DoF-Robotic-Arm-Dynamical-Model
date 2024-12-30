# Import necessary libraries
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import (
    JointGroupCommand,
    JointSingleCommand,
    JointTrajectoryCommand
)

from interbotix_xs_msgs.srv import (
    OperatingModes,
)

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
import time
import math
import socket
import numpy as np
from scipy.linalg import block_diag
import json
import atexit



from Gravity_Compensation_Function import calculate_gravity



class ViperX300sController(Node):
    def __init__(self):
        super().__init__('ViperX_300s_Controller')

        # Call the service to set the operating mode
        self.call_service()

        # Create publisher for joint commands
        self.publisher = self.create_publisher(JointGroupCommand, '/vx300s/commands/joint_group', 10)

        # Subscribe to joint states topic
        self.subscription = self.create_subscription(JointState, '/vx300s/joint_states', self.listener_callback, 10)

        # Store the most recent message without processing immediately
        self.recent_joint_state = None

        # For saving data once the program is finished
        atexit.register(self.save_data)
        self.joint_data = [] #For appending joint data entry



        # Timer to process joint states and publish commands at a slower rate (200Hz in this case)
        self.timer = self.create_timer(0.005, self.process_joint_state)
        self.dt_v = 0.2 # ERG applied reference update time 
        self.timer_ERG = self.create_timer(self.dt_v, self.ERG)  # Call ERG to update the applied reference every dt_v seconds


        # Initialize variables for PD control and joint currents
        self.prev_error = [0] * 9
        self.joint_currents = [0] * 9
        self.PD_command = [0] * 9



        # Proportional (Kp) and derivative (Kd) gains for PID control, tuned for each joint


        self.kp = {
            'waist': 4500,
            'shoulder': 5000,
            'elbow': 4700,
            'forearm_roll': 2200,
            'wrist_angle': 1500,
            'wrist_rotate':1000,
        }

        self.kd = {
            'waist': 200,
            'shoulder': 350,
            'elbow': 200,
            'forearm_roll': 10,
            'wrist_angle': 50,
            'wrist_rotate': 25,
        }         







        # Limits for joint currents
        self.u_min = [-3200.0] * 9
        self.u_max = [3200.0] * 9

        # The desired reference qr
        self.desired_q = {
            'waist': math.radians(90),
            'shoulder': 0.0,
            'elbow': math.radians(-60),
            'forearm_roll': 0.0,
            'wrist_angle': math.radians(60),
            'wrist_rotate': 0.0,
        }


        # Applying angle offsets by converting q to theta (theta_r)
        self.th1_d = self.desired_q['waist']
        self.th2_d = self.desired_q['shoulder'] - math.radians(78.69)
        self.th3_d = self.desired_q['elbow'] - math.radians(11.31)
        self.th4_d = self.desired_q['forearm_roll'] 
        self.th5_d = self.desired_q['wrist_angle'] 
        self.th6_d = self.desired_q['wrist_rotate'] 
        


        # Variable to store the time of the last update
        self.previous_time = time.time()

        #  ERG Parameters:
        self.inc = 0
        
        self.KpMatrix = np.diag([self.kp['waist'], self.kp['shoulder'], self.kp['elbow'], self.kp['forearm_roll'], self.kp['wrist_angle'], self.kp['wrist_rotate']]) # Defined for computing the Lyapunov function
        self.d1 = math.radians(80) #theta1 should be lower than 80 degrees
        self.c1 = np.array([-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]).T # The constraint on the first joint angle
        self.r1 = self.th1_d # The desired reference theta1_r
        self.d3 = math.radians(50 + 11.31) #theta3 should be greater than -61.31 degrees (q3 > -50 degrees)
        self.c3 = np.array([0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]).T # The constraint on the third joint angle
        self.r3 = self.th3_d # The desired reference theta3_r
        self.eta1 = 0.01 #smoothing factor
        self.zeta = math.radians(0.8) # The distance from the constraint from which the repulsive term's effect begins
        self.delta = math.radians(0.5) # The distance from the constraint from which the repulsive term's effect is maximum
        self.kappa = []  # initialize kappa as an empty list
        self.kappa.append(0.0)
        self.th_v1 = []  # initialize v as an empty list
        self.th_v1.append(-30) # At the beginning, the manipulator will go to another location and then it will start ERG. To avoid technical problems, th_v1 is initially set to be the same as the initial state of joint 1
        self.th_v3 = []  # initialize v as an empty list
        self.th_v3.append(math.radians(50)- math.radians(11.31)) # At the beginning, the manipulator will go to another location and then it will start ERG. To avoid technical problems, th_v3 is initially set to be the same as the initial state of joint 3


        self.Running_time = 0 #Initialized to wait for a certain time before starting ERG



    def call_service(self):
        # Service client to set the operating modes of the robotic arm
        client = self.create_client(OperatingModes, '/vx300s/set_operating_modes')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...') # Wait for the service to be available
        
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

        # Extract Joint states
        joint_names = self.recent_joint_state.name
        joint_positions = self.recent_joint_state.position
        joint_velocities = self.recent_joint_state.velocity
        joint_efforts = self.recent_joint_state.effort

        # Calculate the time elapsed since the last update
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time  # update previous time

        # If the running time is more than a certain number of seconds switch to ERG
        self.Running_time = self.Running_time + dt

        if self.Running_time >= 5:

 
            # Reference obtained from ERG. The robot measures q, the reference command should be q
            self.desired_positions = {
                'waist': self.th_v1[-1],
                'shoulder': self.th2_d + math.radians(78.69),
                'elbow': self.th_v3[-1] + math.radians(11.31),
                'forearm_roll': self.th4_d,
                'wrist_angle': self.th5_d,
                'wrist_rotate': self.th6_d,
            }


        else:
            # Go to the initial desired position before starting ERG. The robot measures q, the reference command should be q
            self.desired_positions = {
                'waist': math.radians(-30),
                'shoulder': self.th2_d + math.radians(78.69),
                'elbow': math.radians(50),
                'forearm_roll': self.th4_d,
                'wrist_angle': self.th5_d,
                'wrist_rotate': self.th6_d,
            }




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


        # PD+G control command
        self.joint_currents[0] = max(min(self.PD_command[0] + float(G[0]), 3200.0), -3200.0)
        self.joint_currents[1] = max(min(self.PD_command[1] + float(G[1]), 3200.0), -3200.0)
        self.joint_currents[2] = max(min(self.PD_command[2] + float(G[2]), 3200.0), -3200.0)
        self.joint_currents[3] = max(min(self.PD_command[3] + float(G[3]), 3200.0), -3200.0)
        self.joint_currents[4] = max(min(self.PD_command[4] + float(G[4]), 3200.0), -3200.0)
        self.joint_currents[5] = max(min(self.PD_command[5] + float(G[5]), 3200.0), -3200.0)


        # Publish joint currents as commands
        jointcommand = JointGroupCommand()
        jointcommand.name = 'arm'

        jointcommand.cmd = [self.joint_currents[0], self.joint_currents[1], self.joint_currents[2], self.joint_currents[3], self.joint_currents[4], self.joint_currents[5]]
        self.publisher.publish(jointcommand)

        
        # Record data every time joint states are recieved
        joint_data_entry = {
            'timestamp': current_time,
            'name': joint_names,
            'position': list(joint_positions),
            'velocity': list(joint_velocities),
            'effort': list(joint_efforts),
            'desired_positions': list(self.desired_positions.values()),
            'Kappa': self.kappa[-1]
        }

        self.joint_data.append(joint_data_entry)



    # Explicit Reference Governor Algorithm, called every dt_v seconds for updating the applied reference
    def ERG(self):
        if self.recent_joint_state is None:
            return  # No data to process yet

        joint_names = self.recent_joint_state.name
        joint_positions = self.recent_joint_state.position
        joint_velocities = self.recent_joint_state.velocity










        #  Defined to call the Mass matrix from matlab on the windows side
        q1 = joint_positions[0]
        q2 = joint_positions[1]
        q3 = joint_positions[2]
        q4 = joint_positions[3]
        q5 = joint_positions[4]
        q6 = joint_positions[5]

        input_values = [q1, q2, q3, q4, q5, q6] # in MATLAB they are converted into theta because the model develped is M(theta)...etc
        input_str = ','.join(map(str, input_values))


        # Set up a TCP/IP client
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('172.28.160.1', 12345))  #  '172.28.160.1' is the IP address of  Windows machine

        # Defined for computing the Lyapunov function
        th1 = q1
        th2 = q2 - math.radians(78.69)
        th3 = q3 - math.radians(11.31)
        th4 = q4
        th5 = q5
        th6 = q6

        th1_dot = joint_velocities[0]
        th2_dot = joint_velocities[1]
        th3_dot = joint_velocities[2]
        th4_dot = joint_velocities[3]
        th5_dot = joint_velocities[4]
        th6_dot = joint_velocities[5] 


        Theta = np.array([th1, th2, th3, th4, th5, th6])
        Theta_dot = np.array([th1_dot, th2_dot, th3_dot, th4_dot, th5_dot, th6_dot])  


        self.inc =self.inc+1


        if self.inc == 1:

            th_v1_0 = th1
            th_v3_0 = math.radians(50 - 11.31)
            self.th_v1 = []
            self.th_v3 = []

            self.kappa = []


            # X_v = np.array([th_v1_0, self.th2_d, th_v3_0, self.th4_d, self.th5_d, self.th6_d, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T  #the equilibrium state
            # X = np.array([Theta[0], Theta[1], Theta[2], Theta[3], Theta[4], Theta[5], Theta_dot[0], Theta_dot[1], Theta_dot[2], Theta_dot[3], Theta_dot[4], Theta_dot[5]]).T
 
 
            # The state and the equilibrium point are simplified like that for getting more accurate results since measurement errors exist
            X_v = np.array([th_v1_0, 0.0, th_v3_0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T  #the equilibrium state
            X = np.array([Theta[0], 0.0, Theta[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T



            client_socket.send(input_str.encode())


            # Receive the result from the server
            result = client_socket.recv(4096).decode()  # Increase buffer size if necessary

            # Split the result into rows for the mass matrix
            rows = result.split('\n')

            # Convert the received matrix rows to a numpy array
            M = np.array([list(map(float, row.split(','))) for row in rows])




            # Create a block diagonal matrix where Kp_matrix is the top-left block and M is the bottom-right block
            P = block_diag(self.KpMatrix, M)
            P = 0.5 * P
            

            # \underline{M} which is the minimum M withing the range of operation
            M_low = np.array([[621.845952686235, -14.4922240943411, -9.49642730946135, 291.694209214697, -3.57528375619562, -4.41237726995619],
                              [-14.4922240943411, 622.386439773763, 474.371802823793, -6.41337771483859, 228.065513463355, 3.46043194239875],
                              [-9.49642730946135, 474.371802823793, 457.479484759386, -5.87028978894022, 226.822487035438, 3.58512489972689],
                              [291.694209214697, -6.41337771483859, -5.87028978894022, 334.386844847397, -1.51313658197833, 52.6959597147789],
                              [-3.57528375619562, 228.065513463355, 226.822487035438, -1.51313658197833, 224.491065451646, 3.89685664907189],
                              [-4.41237726995619, 3.46043194239875, 3.58512489972689, 52.6959597147789, 3.89685664907189, 111.611869849729]])
           

            # \underline{P}
            Pc = block_diag(self.KpMatrix, M_low)
            Pc = 0.5 * Pc

            # Threshold value
            gamma1 = ((self.c1.T @ X_v + self.d1)**2) / (self.c1.T @ np.linalg.inv(Pc) @ self.c1)
            gamma3 = ((self.c3.T @ X_v + self.d3)**2) / (self.c3.T @ np.linalg.inv(Pc) @ self.c3)
            gamma  = min(gamma1,gamma3)



            # Compute Lyapunov function 
            Lyapunov = (X-X_v).T @ P @ (X-X_v)

            # Dynamic Safety Margin
            DSM = max(gamma - Lyapunov, 0)
            rho_a1 = (self.r1 - th_v1_0) / max(np.linalg.norm(self.r1 - th_v1_0), self.eta1) # Attraction term for the reference of the first joint
            rho_r1 = max(0, (self.zeta - self.c1.T @ X_v - self.d1) / (self.zeta - self.delta)) * self.c1[0] / np.linalg.norm(self.c1[0]) # Repulsive term for the constraint on the first joint

            # Attraction Field for Joint 1
            Attraction_Field1 = rho_a1 + rho_r1
            g1 = DSM * Attraction_Field1

            rho_a3 = (self.r3 - th_v3_0) / max(np.linalg.norm(self.r3 - th_v3_0), self.eta1) # Attraction term for the reference of the third joint
            rho_r3 = max(0, (self.zeta - self.c3.T @ X_v - self.d3) / (self.zeta - self.delta)) * self.c3[2] / np.linalg.norm(self.c3[2]) # Repulsive term for the constraint on the third joint

            # Attraction Field for Joint 3
            Attraction_Field3 = rho_a3 + rho_r3
            g3 = DSM * Attraction_Field3


            kappa_val  =   1.2e-3

            self.kappa.append(kappa_val)
            th_v1_dot = self.kappa[-1] * g1 # -1 refers to the last element
            self.th_v1.append(th_v1_0 + th_v1_dot * self.dt_v) # update the applied reference for joint 1
            th_v3_dot = self.kappa[-1] * g3 # -1 refers to the last element
            self.th_v3.append(th_v3_0 + th_v3_dot * self.dt_v) # update the applied reference for joint 3

        else:

            client_socket.send(input_str.encode())


            # Receive the result from the server
            result = client_socket.recv(4096).decode()  # Increase buffer size if necessary

            # Split the result into rows for the mass matrix
            rows = result.split('\n')

            # Convert the received matrix rows to a numpy array
            M = np.array([list(map(float, row.split(','))) for row in rows])

            # Create a block diagonal matrix where Kp_matrix is the top-left block and M is the bottom-right block
            P = block_diag(self.KpMatrix, M)
            P = 0.5 * P
            # \underline{M} which is the minimum M withing the range of operation
            M_low = np.array([[621.845952686235, -14.4922240943411, -9.49642730946135, 291.694209214697, -3.57528375619562, -4.41237726995619],
                              [-14.4922240943411, 622.386439773763, 474.371802823793, -6.41337771483859, 228.065513463355, 3.46043194239875],
                              [-9.49642730946135, 474.371802823793, 457.479484759386, -5.87028978894022, 226.822487035438, 3.58512489972689],
                              [291.694209214697, -6.41337771483859, -5.87028978894022, 334.386844847397, -1.51313658197833, 52.6959597147789],
                              [-3.57528375619562, 228.065513463355, 226.822487035438, -1.51313658197833, 224.491065451646, 3.89685664907189],
                              [-4.41237726995619, 3.46043194239875, 3.58512489972689, 52.6959597147789, 3.89685664907189, 111.611869849729]])
           
            # \underline{P}
            Pc = block_diag(self.KpMatrix, M_low)
            Pc = 0.5 * Pc

            
            
            # X_v = np.array([self.th_v1[-1], self.th2_d, self.th_v3[-1], self.th4_d, self.th5_d, self.th6_d, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T  #the equilibrium state
            # X = np.array([Theta[0], Theta[1], Theta[2], Theta[3], Theta[4], Theta[5], Theta_dot[0], Theta_dot[1], Theta_dot[2], Theta_dot[3], Theta_dot[4], Theta_dot[5]]).T

            # The state and the equilibrium point are simplified like that for getting more accurate results since measurement errors exist
            X_v = np.array([self.th_v1[-1], 0.0, self.th_v3[-1], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T  #the equilibrium state
            X = np.array([Theta[0], 0.0, Theta[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
            
            # Compute Lyapunov function           
            Lyapunov = (X-X_v).T @ P @ (X-X_v)

            # Threshold value
            gamma1 = ((self.c1.T @ X_v + self.d1)**2) / (self.c1.T @ np.linalg.inv(Pc) @ self.c1)
            gamma3 = ((self.c3.T @ X_v + self.d3)**2) / (self.c3.T @ np.linalg.inv(Pc) @ self.c3)
            gamma  = min(gamma1,gamma3)



 



            # Dynamic Safety Margin
            DSM = max(gamma - Lyapunov, 0)

            rho_a1 = (self.r1 - self.th_v1[-1]) / max(np.linalg.norm(self.r1 - self.th_v1[-1]), self.eta1) # Attraction term for the reference of the first joint
            rho_r1 = max(0, (self.zeta - self.c1.T @ X_v - self.d1) / (self.zeta - self.delta)) * self.c1[0] / np.linalg.norm(self.c1[0])# Repulsive term for the constraint on the first joint


            # Attraction Field for Joint 1
            Attraction_Field1 = rho_a1 + rho_r1
            g1 = DSM * Attraction_Field1

            rho_a3 = (self.r3 - self.th_v3[-1]) / max(np.linalg.norm(self.r3 - self.th_v3[-1]), self.eta1) # Attraction term for the reference of the third joint
            rho_r3 = max(0, (self.zeta - self.c3.T @ X_v - self.d3) / (self.zeta - self.delta)) * self.c3[2] / np.linalg.norm(self.c3[2]) # Repulsive term for the constraint on the third joint


            # Attraction Field for Joint 3
            Attraction_Field3 = rho_a3 + rho_r3
            g3 = DSM * Attraction_Field3

            kappa_val  =   1.2e-3
            # self.get_logger().info(f'kappa_val: {kappa_val}')

            self.kappa.append(kappa_val)
            th_v1_dot = self.kappa[-1] * g1 # -1 refers to the last element
            self.th_v1.append(self.th_v1[-1] + th_v1_dot * self.dt_v) # update the applied reference for joint 1

            th_v3_dot = self.kappa[-1] * g3 # -1 refers to the last element
            self.th_v3.append(self.th_v3[-1] + th_v3_dot * self.dt_v) # update the applied reference for joint 3
            
        
        


        client_socket.close()



    # Save all data entery once the programe is stopped
    def save_data(self):
        with open('ERG_2_data_constantKappa1.json', 'w') as f:
            json.dump(self.joint_data, f, indent=4)
        self.get_logger().info('Joint data saved.json')



# Entry point of the script
def main(args=None):
    rclpy.init(args=args) # Initialize ROS2
    viperx_300s_controller = ViperX300sController() # Create the controller object
    rclpy.spin(viperx_300s_controller) # Keep the node running

    # Clean up resources
    viperx_300s_controller.destroy_node()
    rclpy.shutdown()


# Run the script
if __name__ == '__main__':
    main()








