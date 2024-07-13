/*
Date:08/11/2023
Developed by: Musa Almaz
Project: Inverse Kinematics
Summary: This header file is for controlling robot with inverse kinematic outcomes.
It creates random points in workspace and by calculating the necessary joint angles(since the robot is RRR) and goes to this point.
The inverse kinematic equations are calculated with the last column of the kinematic map.
*/ 

#ifndef INVERSE_KINEMATICS_HPP
#define INVERSE_KINEMATICS_HPP

#include "sphere_spawner.hpp"
#include <cstdlib>
#include <ctime>
#include <vector>
#include <random>
#include <cmath>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/LinkStates.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <std_msgs/Float64.h>

// Define the sphere_pose struct to put random points
struct sphere_pose {
    double x;
    double y;
    double z;
};

class InverseKinematics{
    public:
        /*
        Date:09/11/2023
        Developed by: Musa Almaz
        Summary: This method is for creating random point in workspace to spawn the spheres.
        Input:No input
        Output:No output
        Additional info: It puts the points in target_vector.
        */ 
        void create_target_points();

        /*
        Date:09/11/2023
        Developed by: Musa Almaz
        Summary: This method uses "sphere_spawner" to spawn the sphers on gazebo.
        Input:No input
        Output:No output
        Additional info: The "target_vector" is used to positioning the spheres .
        */ 
        void spawn_sphere_to_gazebo();
        
        /*
        Date:09/11/2023
        Developed by: Musa Almaz
        Summary: this is the main function to move robot with inverse kinematic.
        Input:No input
        Output:No output
        Additional info: This method uses "calculateDistance" and "isTouching" methods.
        */ 
        void move_robot();

            /*
        Date:09/11/2023
        Developed by: Musa Almaz
        Summary: Constructor for the SpawnSphere class.
        Input:   - nh_: A ROS NodeHandle for initializing ROS-related components.
        Output:  No explicit output.
        */
        InverseKinematics(const ros::NodeHandle& nh_);

        ros::NodeHandle nh;

        // Define the number of target, it is changable with command line input
        int number_of_target = 5;


    private:
        /*
        Date:09/11/2023
        Developed by: Musa Almaz
        Summary: This callback is used to get the position of the tip
        Input: It takes the gaebo message for the position of links.
        Output:No output
        Additional info: Position of the link is used to detect touching the sphere.
        */ 
        void LinkStatesCb(const gazebo_msgs::LinkStatesConstPtr &msg);

        /*
        Date:09/11/2023
        Developed by: Musa Almaz
        Summary: this method returns the distance between target point and tip position.
        Input: It takes a position defined by sphere_pose construct.
        Output:Distance between the target point and tip posiion.
        Additional info: It also uses the position of the tip.
        */ 
        double calculateDistance(const sphere_pose& a);
        
        /*
        Date:09/11/2023
        Developed by: Musa Almaz
        Summary: This method return "true" if the distance between tip position and center of the sphere is lower than the threshold.
        Input:It takes a position defined by sphere_pose construct.
        Output:No output
        Additional info:
        */ 
        bool isTouching(const sphere_pose& point);
        // Threshold for touching the sphere
        double threshold = 0.03; //meter

        const char tip[11] = "robot::tip";

        std::vector<sphere_pose> target_vector;  // Use std::vector

        // define the publisher and subscribers
        ros::Publisher joint1_publisher;
        ros::Publisher joint2_publisher;
        ros::Publisher joint3_publisher;
        ros::Subscriber tip_position_subscriber;

        geometry_msgs::Point tip_position;
        geometry_msgs::Pose sphere_position;
        geometry_msgs::Pose changeColorPose;

        int ros_rate = 10;

        sphere_pose home_pose = {0, 0, 2}; // Home position of tip of the robot
        sphere_pose center = {0, 0, 1}; // Center of the sphere
        double radius = 1; // Radius of the sphere

        double joint1_angle;
        double joint2_angle;
        double joint3_angle;

        std_msgs::Float64 joint1_command_value, joint2_command_value, joint3_command_value;
        
        // define the random variables for the position of spheres
        double x_position;
        double y_position;
        double z_position;

        // Create the object to use this class
        SpawnSphere sphereSpawner;

        // assume l3 = l2 = l1 = 0.5
        double link_length = 0.5;
 

};



#endif // INVERSE_KINEMATICS_HPP

