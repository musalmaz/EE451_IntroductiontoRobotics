Put this package under src folder of your workspace. Then, build the worspace. Source the setup.bash file under Devel folder of the workspace.
To spawn the robot, run the controllers and type:
roslaunch rrrbot spawn_robot.launch

To run the task 3 which puts the spheres on the robot's workspace, type:
rosrun rrrbot visualize_workspace 

To run the task 4 which uses inverse kinematic to touch the spheres, type:
rosrun rrrbot touch <N>
N i sthe number of spheres and the default value for this is 5, so if you don't put any parameter, there will be 5 sphere.

I changed the launch file, so it is not runs the random_kinematics node. 
