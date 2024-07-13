/*
Date:19/12/2023
Developed by: Musa Almaz
Project: Path Planning using APF
Summary: This main file is the necessary function calls 
for the obstacle avoidance and path planning task
*/ 
#include "robot_controller.hpp"

#define ROS_RATE 20

int main(int argc, char ** argv){
    ros::init(argc, argv, "main_task2");
    ros::NodeHandle nh;
    // Loop to update subcribers
    ros::Rate control_rate(ROS_RATE);
    for(int i = 0; i< 10; i++){
        ros::spinOnce();
        control_rate.sleep();
    }
    // Create the object
    RobotController robot(nh);
    // Print the current position and goal position
    robot.print();
    // Do the main task
    robot.move_robot();
    // Print the current position
    robot.print();
    while (ros::ok())
    {
        ros::spinOnce();
        control_rate.sleep();
    }
    
}