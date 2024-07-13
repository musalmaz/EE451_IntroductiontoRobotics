/*
Date:29/11/2023
Developed by: Musa Almaz
Project:  (Project 3) velocity kinematics and differential movements 
Summary: This file is used to make necessary function calls.
The task is moving robot with given joint velocities.
*/

#include "velocity_kinematics.hpp"

#define ROS_RATE 10

int main(int argc, char ** argv){

    ros::init(argc, argv, "test_velocity_kinematics");
    ros::NodeHandle nh;

    VelocityKinematics velKinematic(nh);

    ros::Rate rate(ROS_RATE);
    for(int i = 0; i < 10; i++){
        ros::spinOnce();
        rate.sleep();
    }
    // Move robot, compute Jacobian, print and save Jacobian, update kinematic map and print position and orientation change by calling necessary functions
    velKinematic.applyVelocity();

    // read the files and calculate the joint velocities, plot the calculated vs given joint velocities.
    velKinematic.calculateInverseKinematic();

    return 0;

}