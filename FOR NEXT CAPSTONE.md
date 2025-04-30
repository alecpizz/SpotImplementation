# For the next capstone project:

[General Notes](https://docs.google.com/document/d/1YLWGcRS-MFEN1-fMZ1ft0pSkrqvDmba6fllMAzXQj9c/edit?usp=sharing)

## Problems for the next group to solve:

This project was hard, and there was a decent amount of engineering done for this simulation. Here's the current problems with the robot:
- Movement is not perfectly straight. The math isn't perfect, and errors accumulate through things like friction and general simulation instability.
- Representing a maze through zero's and one's is hard to keep detail. How do you represent a tile with walls above it, with no walls to the sides? And then run that said representation through a traditional maze solving algorithm. The maze is generated in runtime, which is a big advantage for iteration however.
- Mazes with paths right next to each other cannot be represented in this style and therefore cannot be solved.
- The maze solver has a lot of magic PID constants.
- The maze solver cannot be sped up with Webot's time speed up function, the time checking logic does not properly increase with it.

## Things the robot does
- Solves the maze given.
- Uses PIDs to balance movement and stay centered/avoid wall collisions.
- Receives most of the commands needed for movement, e.g. forward, left, right, back, as well as strafing and blending between movements.
- Uses LIDAR to detect wall depth.
- Procedurally generates movement steps and trajectories.

## Things you should learn/know
- How LIDAR works in Webots.
- How PID controllers work.
- What Inverse Kinematics (IK) is and how a basic trigonometry implementation of that works.
- General Webots and Python knowledge.
- AVOID ROS2 IF YOU VALUE YOUR TIME. It'll only be really useable on linux. 