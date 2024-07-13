/*
Date:19/12/2023
Developed by: Musa Almaz
Project: Path Planning using APF
Summary: This header file is for plannning a path using APF and avoiding obstacles.
*/ 

#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <array>
#include <vector>
#include <geometry_msgs/Point.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SpawnModel.h>

//Define the struct for obstacles
struct Obstacle{
    double pos_x;
    double pos_y;
    double radius;
    double height;
};

class RobotController{
    public:
        /*
        Date: 19.12.2023
        Developed by: Musa Almaz
        Summary: This is the basic function for moving the robot.
        Input: no input 
        Output: no output
        Additional info: This publishes the velocity commands
        */ 
        void move_robot();
        /*
        Date: 19.12.2023
        Developed by: Musa Almaz
        Summary: This is for printing current robot position
        Input: no input 
        Output: no output
        Additional info: -
        */ 
        void print();
        /*
        Date:19/12/2023
        Developed by: Musa Almaz
        Summary: Constructor for the RobotController class.
        Input:   No explicit input.
        Output:  No explicit output.
        */
        RobotController(const ros::NodeHandle& nh_);
        ros::NodeHandle nh;

    private:
        /*
        Date:19/12/2023
        Developed by: Musa Almaz
        Summary: Calculates the output of the APF result.
        Input:   No explicit input.
        Output:  Returns the APF result.
        */
        std::array<double,2> velocity_APF();
        /*
        Date:19/12/2023
        Developed by: Musa Almaz
        Summary: File reader function for reading the obstacle infos and goal position.
        Input:   No explicit input.
        Output:  No explicit output.
        */
        void file_reader();
        /*
        Date:19/12/2023
        Developed by: Musa Almaz
        Summary: Creates the string format of a cylinder as SDF model.
        Input:   obstace infos consisting of position, radius and heigth of the cylinder and id of the obstacle.
        Output:  String definition of the related object.
        */
        std::string create_cylinder_SDF(const Obstacle& obstacle, int id);
        /*
        Date:19/12/2023
        Developed by: Musa Almaz
        Summary: This is for spawning the obstacles in the Gazebo envionment.
        Input:   No explicit input.
        Output:  No explicit output.
        Additional info: Uses the output of the "create_cylinder_SDF" function.
        */
        void spawn_obstacles_in_gazebo();
        /*
        Date:19/12/2023
        Developed by: Musa Almaz
        Summary: This is the callback function for positin and orientation of the robot.
        Input:   address of the msg that contains the position and orientatin
        Output:  No explicit output.
        */
        void ModelStatesCb(const gazebo_msgs::ModelStatesConstPtr &msg);

        geometry_msgs::Point current_robot_position;
        geometry_msgs::Quaternion robot_orientation;

        // Define the file path
        std::string obstacle_file = ros::package::getPath("turtlebot") + "/config/robot_task.txt";
        std::array<double,2> goal_position;
        std::vector<Obstacle> obstacle_info;

        // Define the obstacle number, initialize as 0, increment in file reading
        int obstacle_number = 0;

        
        ros::Subscriber robot_position_subscriber;
        ros::Publisher command_velocity_publisher;


        const double robot_radius = 0.16; // m

        double vel_x;
        double vel_y;

        geometry_msgs::Twist command_vel;

        const int ROS_RATE = 20;
        // Define and initialize the APF gains
        const double ATTRACTIVE_GAIN = 2.63; 
        const double REPULSIVE_GAIN = 0.52;  

        // Define and initialize the velocity limits
        const double max_lin_vel = 0.3;
        const double min_lin_vel = -0.3;

        const double max_angular_vel = 2.0;
        const double min_angular_vel = -2.0;

        // Define and initialize the threshold for closing the goal position
        double threshold = 0.2;
};



#endif // ROBOT_CONTROLLER_HPP