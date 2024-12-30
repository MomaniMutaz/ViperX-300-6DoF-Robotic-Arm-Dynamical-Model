# ViperX-300-6DoF-Robotic-Arm-Dynamical-Model
This repository contains a physically feasible dynamical model of the ViperX-300 6DoF robotic arm, designed to aid researchers, engineers, and enthusiasts in understanding and controlling the manipulator. The model accounts for the following key components:

Mass Matrix (M): Capturing the inertia properties of the arm.
Coriolis Vector (V): Representing the Coriolis and centrifugal forces.
Gravity Compensation Vector (G): Providing the torques required to counteract gravitational forces.
Friction Torque Vector: Accounting for viscous and Coulomb friction.
Torque Offsets: Addressing any system-specific torque discrepancies.
The model is implemented in MATLAB and Python, making it accessible for users in both environments. It enables users to:

Simulate the arm's dynamics.
Develop model-based controllers, such as gravity compensation or computed torque control.
This repository includes:

A Python script for controlling the ViperX-300 6DoF robotic arm using ROS2. The controller performs gravity compensation and implements Proportional-Derivative (PD) control for precise joint motion.
Code for an Explicit Reference Governor (ERG), designed to enforce linear constraints dynamically during robotic operations.
A server-client setup to call the mass matrix computation, enabling efficient communication between the Python controller and MATLAB functions.

This repository aims to support robotic system development and research with a robust, physically feasible dynamical model.  Feel free to explore the code and contribute to the project!
