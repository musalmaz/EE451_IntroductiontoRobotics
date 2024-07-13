/*
Date:08/11/2023
Developed by: Musa Almaz
Project: Inverse Kinematics
Summary: This header file is for creating spheres in gazebo environment. 
*/ 

#ifndef SPHERE_SPAWNER_HPP
#define SPHERE_SPAWNER_HPP

#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <geometry_msgs/Pose.h>
#include <string>

class SpawnSphere {
public:
    /*
    Date:09/11/2023
    Developed by: Musa Almaz
    Summary: Spawns a sphere model in Gazebo with the given pose.
    Input:   - spherePose: The pose (position and orientation) of the sphere to be spawned.
    Output:  No explicit output.
    */
    void spawnSphereToGazebo(const geometry_msgs::Pose& spherePose);

    /*
    Date:09/11/2023
    Developed by: Musa Almaz
    Summary: Changes the color of a sphere model in Gazebo with the given pose.
    Input:   - spherePose: The pose (position and orientation) of the sphere to change color.
             - model_name: The name of the sphere model to change color.
    Output:  No explicit output.
    */
    void changeSphereColor(const geometry_msgs::Pose& spherePose, const std::string& model_name);

    /*
    Date:09/11/2023
    Developed by: Musa Almaz
    Summary: Publishes the pose of the sphere to a ROS topic.
    Input:   No explicit input.
    Output:  No explicit output.
    */
    void spherePosePublisher();

    /*
    Date:09/11/2023
    Developed by: Musa Almaz
    Summary: Constructor for the SpawnSphere class.
    Input:   - nh_: A ROS NodeHandle for initializing ROS-related components.
    Output:  No explicit output.
    */
    SpawnSphere(const ros::NodeHandle& nh_);

private:
    /*
    Date:09/11/2023
    Developed by: Musa Almaz
    Summary: This method creates the sdf file of the sphere with the given color
    Input:   The color of the sphere
    Output:  The sdf model of a simple sphere
    */
    std::string createSphereSdf(const std::string& color) {
    return R"(
        <sdf version='1.6'>
            <model name='sphere_model'>
                <pose>0 0 0 0 0 0</pose>
                <link name='link'>
                    <gravity>false</gravity>
                    <visual name='visual'>
                        <geometry>
                            <sphere><radius>0.05</radius></sphere>
                        </geometry>
                        <material>
                            <ambient>)" + color + R"(</ambient>
                            <diffuse>)" + color + R"(</diffuse>
                            <specular>0.1 0.1 0.1 1</specular>
                            <emissive>0 0 0 0</emissive>
                        </material>
                    </visual>
                </link>
            </model>
        </sdf>
        )";
    }
    static int count; // Static variable to keep track of the number of instances of the SpawnSphere class.

    geometry_msgs::Pose spherePose; // Stores the pose of the sphere.

    ros::Publisher publishSpherePose; // Publisher for sending sphere pose information to a ROS topic.

    gazebo_msgs::SpawnModel spawnModelRequest; // Request object for spawning a Gazebo model.

    ros::ServiceClient spawnClient; // ROS service client for spawning Gazebo models.

    ros::ServiceClient delete_client_; // ROS service client for deleting Gazebo models.

    ros::NodeHandle nh; // ROS NodeHandle for managing ROS-related operations.

    std::string Red = "1 0 0 1"; // String representing the color red in RGBA format.

    std::string Green = "0 1 0 1"; // String representing the color green in RGBA format.


};

#endif // SPHERE_SPAWNER_HPP
