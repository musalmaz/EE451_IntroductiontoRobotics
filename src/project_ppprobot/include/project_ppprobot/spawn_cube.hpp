#ifndef SPAWN_CUBE_HPP
#define SPAWN_CUBE_HPP

#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>
class Spawn_Cube {
    public:
        /*
        Date: 17.10.2023
        Developed by: Musa Almaz
        Summary: This is to spawn the cube to the gazebo
        Input: no input
        Output: no output
        Additional info: This contains publisher initializations
        */ 
        void spawn_cube_to_gazebo();
        /*
        Date: 17.10.2023
        Developed by: Musa Almaz
        Summary: Constructpor of the class
        Input: no input
        Output: no output
        Additional info: 
        */
        Spawn_Cube(const ros::NodeHandle& nh_);
        /*
        Date: 17.10.2023
        Developed by: Musa Almaz
        Summary: This is to publish the position of the cube
        Input: no input
        Output: no output
        Additional info: This publishes the data over the "cube_pose" topic
        */
        void cube_pose_publisher();
        // Definition and initialization of a variable typed ros::NodeHandle
        ros::NodeHandle nh;
    private:
        // Define the limits of the space where the cube can be spawned
        double xMin = 0.5;
        double xMax = 1.0;
        double yMin = 0.5;
        double yMax = 1.0;
        double z = 0.05;
        // Get the path of the home
        const char* home = getenv("HOME");
        // Reach the path of the urdf file of the cube
        std::string urdfFilePath = std::string(home) + "/EE451/src/project_ppprobot/urdf/cube.urdf";
        // Define the seed for random number generation
        std::time_t time_seed;
        unsigned int seed;
        
        // define the random variables
        double random_x;
        double random_y;
        // Define the position of the cube
        geometry_msgs::Pose cubePose;
        // define the publisher to publish the position of the cube
        ros::Publisher publishCubePose;
        // Create a request to spawn the cube
        gazebo_msgs::SpawnModel spawnModelRequest;
 
        // Create a client for the SpawnModel service
        ros::ServiceClient spawnClient;

};

#endif