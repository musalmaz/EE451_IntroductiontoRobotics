#ifndef CONTROL_ROBOT_HPP
#define CONTROL_ROBOT_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <random>
// for the high_resolution_clock
#include <chrono> 
class Control_Robot{
    public:
        /*
        Date: 18.10.2023
        Developed by: Musa Almaz
        Summary: This is the constructor of the class
        Input: ros::NodeHandle
        Output: no output
        Additional info: This contains publisher initializations
        */ 
        Control_Robot(const ros::NodeHandle& nh_);
        /*
        Date: 18.10.2023
        Developed by: Musa Almaz
        Summary: The aim of the function is to translate the cube.
        Input: no input
        Output: no output
        Additional info: This function includes other function calls in a switch case loop to randomly translate the cube
        */ 
        void move_cube();
       
        /*
        Date: 18.10.2023
        Developed by: Musa Almaz
        Summary: This is used for translating the cube in the positive x direction
        Input: no input
        Output: no output
        Additional info: Called by move_cube function
        */ 
        void move_forward_x();
        /*
        Date: 18.10.2023
        Developed by: Musa Almaz
        Summary: This is used for translating the cube in the negative x direction
        Input: no input
        Output: no output
        Additional info: Called by move_cube function
        */ 
        void move_backward_x();
        /*
        Date: 18.10.2023
        Developed by: Musa Almaz
        Summary: This is used for translating the cube in the positive y direction
        Input: no input
        Output: no output
        Additional info: Called by move_cube function
        */ 
        void move_forward_y();
        /*
        Date: 18.10.2023
        Developed by: Musa Almaz
        Summary: This is used for trasnlating the cube in the negative y direction
        Input: no input
        Output: no output
        Additional info: Called by move_cube function
        */ 
        void move_backward_y();
        /*
        Date: 18.10.2023
        Developed by: Musa Almaz
        Summary: This is the callback function of the subscriber
        Input: Address of the position of the cube
        Output: no output
        Additional info:
        */ 
        void cubePoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
        /*
        Date: 18.10.2023
        Developed by: Musa Almaz
        Summary: This is to copy the position of the cube to another variable
        Input: no input
        Output: no output
        Additional info: This includes the initialization of the subcriber
        */ 
        void get_cube_pose();
        // Definition and initialization of a variable typed ros::NodeHandle
        ros::NodeHandle nh;
    private:
        // Definition of the position of the cube varibles
        geometry_msgs::Pose cubePosition;
        geometry_msgs::Pose copy_cubePosition;
        // Definition of the subscriber to get the position of the cube
        ros::Subscriber sub_cube_pose;

        // Define the joint publishers
        ros::Publisher z_joint_pub;
        ros::Publisher y_joint_pub;
        ros::Publisher x_joint_pub;
        // Check the callback is called
        bool callback_received;
        // Initialize the ros rate that is used inside the functions
        int ros_rate = 10;
        // Define the distance between the cube and end effector
        float x_distance_to_cube;
        float y_distance_to_cube;
        // Define the initial position of the end effector
        const float initial_x_pose = 0.55;
        const float initial_y_pose = 0.55;
        // Define the half size of the cube
        const float cube_half_edge = 0.05;
        // Define and initialize the robot downward movement
        const float down_along_z = -0.15;
        // Initialize a tolerance value to avoid collision
        const float tolerance = 0.05;
        // Define and initialize the wanted translation
        const float translation_distance = 0.1;
        // Define the seed to generate random numbers
        unsigned seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        // Define a random integer
        int random_integer;

};

#endif