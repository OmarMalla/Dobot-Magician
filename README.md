# Dobot-Magician
A simplified model to accurately study the Dobot Magician kinematics.
- Dobot Simulation with Geometric approach and the **DH Apporach** based on the *simplified model*.
- Python Code to move the robot to different poses and save them.

If found helpful, kindly consider referencing:
*Malla, O. and Shanmugavel, M. (2024), "Simplified model to study the kinematics of manipulators with parallelogram linkages", Industrial Robot, Vol. 51 No. 5, pp. 704-714. https://doi.org/10.1108/IR-01-2024-0046*

The code requires the use of Peter Corke's Robotics Toolbox for Matlab:
https://petercorke.com/toolboxes/robotics-toolbox/

THe Matlab code contains:
1) Robot Specs as per the real robot.
2) Geometric Kinematic solution.
3) Simplified model with DH parametrization.
4) Comparison to the vlaues captured from Dobot Studio for
joints rotations across the workspace.
5) Test cases captured from the robot's hardware.
6) Error and Workspace Plotting.
7) Functions to calculate Inverse Kinematics analytically and numerically.

The attached files contain different poses of the Dobot with respect to individual joint variable changes.
