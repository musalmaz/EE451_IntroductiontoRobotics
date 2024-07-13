/*
Date:08/11/2023
Developed by: Musa Almaz
Project: Inverse Kinematics
Summary: This source file is for creating spheres in gazebo environment.
It consists of the source codes of "sphere-spawner.hpp".
*/ 

#include "sphere_spawner.hpp"

// Initialize the count
int SpawnSphere::count = 0;

SpawnSphere::SpawnSphere(const ros::NodeHandle& nh_) : nh(nh_) {
    // Initialize the publisher and service client
    publishSpherePose = nh.advertise<geometry_msgs::Pose>("sphere_pose", 10);
    spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    delete_client_ = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
}

void SpawnSphere::spawnSphereToGazebo(const geometry_msgs::Pose& spherePose) {
    // Set the fields in the spawn model request
    spawnModelRequest.request.model_name = "sphere" + std::to_string(count++);
    spawnModelRequest.request.model_xml = createSphereSdf(Red);
    spawnModelRequest.request.initial_pose = spherePose;

    // Call the service to spawn the model
    if (!spawnClient.call(spawnModelRequest)) {
        ROS_ERROR("Failed to call service /gazebo/spawn_sdf_model");
    }
}

void SpawnSphere::spherePosePublisher() {
    // Publish the sphere pose
    publishSpherePose.publish(spherePose);
}

void SpawnSphere::changeSphereColor(const geometry_msgs::Pose& spherePose, const std::string& model_name){
    // Delete the old model
    gazebo_msgs::DeleteModel delete_model;
    delete_model.request.model_name = model_name;
    delete_client_.call(delete_model);
    ros::Duration(1).sleep();

    spawnModelRequest.request.model_name = model_name;
    spawnModelRequest.request.model_xml = createSphereSdf(Green);
    spawnModelRequest.request.initial_pose = spherePose;

    // Call the service to spawn the model
    if (!spawnClient.call(spawnModelRequest)) {
        ROS_ERROR("Failed to call service /gazebo/spawn_sdf_model");
    }

}
