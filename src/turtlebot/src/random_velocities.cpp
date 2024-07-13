/*
Date:19/12/2023
Developed by: Musa Almaz
Project: Estimating the position
Summary: This main file is for estimatig the odometry 
of the robot using left and right wheel velocities.
*/ 
#include "../include/to_euler.h"
#include <fstream>
#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/package.h>
#include <chrono>

#define WHEEL_SEPARATION 0.26
#define WHEEL_DIAMETER 0.072
#define MAX_LINEAR_VELOCITY 1
#define MIN_LINEAR_VELOCITY -1
#define MAX_ANGULAR_VELOCITY 1
#define MIN_ANGULAR_VELOCITY -1

geometry_msgs::Point robot_position;
geometry_msgs::Point robot_orientation;

void ModelStatesCb(const gazebo_msgs::ModelStatesConstPtr &msg) {
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "robot") {
            robot_position = msg->pose[i].position;
            robot_orientation = ToEulerAngle(msg->pose[i].orientation);
        }
    }
}
double left_wheel_velocity, right_wheel_velocity;

void LinkStatesCb(const gazebo_msgs::LinkStatesConstPtr &msg) {
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "robot::left_wheel_link") {
            left_wheel_velocity = cos(robot_orientation.z) * msg->twist[i].angular.y -
                                  sin(robot_orientation.z) * msg->twist[i].angular.x;
        } else if (msg->name[i] == "robot::right_wheel_link") {
            right_wheel_velocity = cos(robot_orientation.z) * msg->twist[i].angular.y -
                                   sin(robot_orientation.z) * msg->twist[i].angular.x;
        }
    }
}

struct RobotState {
    double x;   // X position
    double y;   // Y position
    double theta; // Heading in radians
};

/*
Date:19/12/2023
Developed by: Musa Almaz
Summary: This is for calculating the position and heading of the robot using wheel velocities.
Input:   Previous state of the robot, left and right velocity of the wheels, duration between previous step and current step.
Output:  No explicit output.
Additional info: updates the state of the robot.
*/
void updateRobotPosition(RobotState &state, double left_wheel_velocity, double right_wheel_velocity, auto duration) {
    // Calculate linear velocity of each wheel
    double left_wheel_linear_velocity = left_wheel_velocity * (WHEEL_DIAMETER / 2.0);
    double right_wheel_linear_velocity = right_wheel_velocity * (WHEEL_DIAMETER / 2.0);


    // Update position
    state.x -= (WHEEL_DIAMETER / 4.0) * sin(state.theta) * (left_wheel_velocity + right_wheel_velocity) * duration;
    state.y += (WHEEL_DIAMETER / 4.0) * cos(state.theta) * (left_wheel_velocity + right_wheel_velocity) * duration;
    state.theta += (right_wheel_linear_velocity - left_wheel_linear_velocity) * duration / WHEEL_SEPARATION;

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "random_kinematics");
    ros::NodeHandle nh;

    ros::Publisher command_velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Subscriber link_states_subscriber = nh.subscribe("/gazebo/link_states", 1, LinkStatesCb);
    ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 1, ModelStatesCb);

    ros::Rate control_rate(20);

    ros::Duration(1).sleep();


    {
        std::ifstream task_file(ros::package::getPath("turtlebot") + "/config/robot_task.txt");
        std::string line;
        double number;
        ROS_INFO("Opening task file,");
        while (getline(task_file, line)) {
            //get target position, obstacles positions
            std::istringstream sin(line);
            while (sin >> number){
                std::cout<<number<<" ";
            }
            std::cout<<"\n";
        }

        task_file.close();
    }

    for(int i = 0; i< 10; i++){
        ros::spinOnce();
        control_rate.sleep();
    }

    RobotState robot = {0, 0, -M_PI/2.0}; // Initial state
    
    std::srand(static_cast<unsigned int>(time(nullptr))); //random number seed

    geometry_msgs::Twist command_vel;
    int counter = 100;
    
    auto last_time = std::chrono::high_resolution_clock::now();
    while (ros::ok()) {

        if (counter == 200) {
            counter = 0;
            command_vel.linear.x =
                    (((double) rand()) / RAND_MAX) * (MAX_LINEAR_VELOCITY - MIN_LINEAR_VELOCITY) + MIN_LINEAR_VELOCITY;

            command_vel.angular.z = 
                    (((double) rand()) / RAND_MAX) * (MAX_ANGULAR_VELOCITY - MIN_ANGULAR_VELOCITY) +
                    MIN_ANGULAR_VELOCITY;
            
            ROS_INFO("Current left-right wheel angular velocities: %f, %f", left_wheel_velocity, right_wheel_velocity);
            ROS_INFO("New velocity targets:  %f, %f \n", command_vel.linear.x, command_vel.angular.z);

        }

        command_velocity_publisher.publish(command_vel);

        counter++;
        ros::spinOnce();
        auto end = std::chrono::high_resolution_clock::now();
        // Calculate the elapsed time
        auto duration = std::chrono::duration<double>(end - last_time).count();

        last_time = end;
        updateRobotPosition(robot, left_wheel_velocity,right_wheel_velocity, duration);

        ROS_INFO("Estimated position : %f ,%f", robot.x, robot.y);
        ROS_INFO("Current robot position: %f, %f", robot_position.x, robot_position.y);
        control_rate.sleep();
    }


    return 0;
}
