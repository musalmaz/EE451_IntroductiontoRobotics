Put this package under src folder of your workspace. Then, build the worspace. Source the setup.bash file under Devel folder of the workspace.
To spawn the robot and run random velocity publisher, type:
roslaunch turtlebot spawn_robot.launch

For the task 1, wh,ch is compaing the estimated ad actual position, type:
rosrun turtlebot random_velocities

For the task 2, which is obstacle avoidance using APF type:
rosrun turtlebot main_task2
