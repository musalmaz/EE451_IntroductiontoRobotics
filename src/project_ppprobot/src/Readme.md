This is the readme file of the project 1.
This project is about controlling a PPP robot.

As an answer to the part b;
Given a PPP robot, the position of the tip of the last link (point p) in the base coordinate frame oxyz
will be determined by the translations of each of the three prismatic joints.
Let's denote the translations of the three prismatic joints as d1, d2 and d3;
1 - The first prismatic joint moves along the x-axis by a distance d1,
2 - The second prismatic joint moves along the y-axis by a distance d2,
3 - The third prismatic joint moves along the z-axis by a distance d3
Then the coordinates of point p with respect to the base coordinate frame:
p(x,y,z) = (d1, d2, d3)
This is a straightforward calculation because, in a PPP robot, each joint simply translates along one of the coordinate axes (assuming the robot is aligned with the coordinate frame). The total movement along each axis is the extension of the corresponding prismatic joint.

- spawn_cube.hpp header includes the necessary initializations to spawn a cube in a random position in gazebo.
- spawn_cube_main.cpp file spawns the cube by calling the spawn_cube.hpp header.

- control_robot.hpp header includes the necessary initializations to control the robot and move the cube.
- main.cpp file calls the necessary function to do the task.

To compile and run;
- $cd EE451
- $catkin_make
then to launch the robot, source the environment and
- $roslaunch project_ppprobot robot.launch 
to spawn the cube, run the following on other terminal:
- $rosrun project_ppprobot spawn
to move the cube in any random direction, run the following in other terminal:
- $rosrun project_ppprobot main
