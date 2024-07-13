/*
Date:08/11/2023
Developed by: Musa Almaz
Project: Inverse Kinematics
Summary: This file is used to touch the sphere in gazebo environment.
The task is touching the spheres and when the tip is touched a spere, the color of the sphere is changing.
*/

#include "inverse_kinematics.hpp"
#define ROS_RATE 10

int main(int argc, char **argv) {

    ros::init(argc, argv, "touch_sphere");
    ros::NodeHandle nh;

    // create the object for InverseKinematics
    InverseKinematics cobot(nh);

    if(argc < 2){
        std::cout <<"Please give number of target. Default target number is 5" << std::endl;
    }else if(argc == 2){
        cobot.number_of_target = std::atoi(argv[1]);
    }

    ros::Rate rate(ROS_RATE);
    for(int i = 0; i < 20; i++){
        ros::spinOnce();
        rate.sleep();
    }
    // create the positions of the spheres
    cobot.create_target_points();
    // spawn spheres is gazebo
    cobot.spawn_sphere_to_gazebo();
    // do task
    cobot.move_robot();
 

    return 0;

}