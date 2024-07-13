/*
Date:08/11/2023
Developed by: Musa Almaz
Project: Inverse Kinematics
Summary: This file is the source file of "inverse_kinematics.hpp".
*/ 

#include "inverse_kinematics.hpp"

InverseKinematics::InverseKinematics(const ros::NodeHandle& nh_) : sphereSpawner(nh){
    this->nh = nh_;
    this->joint1_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint1_controller/command", 10);
    this->joint2_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint2_controller/command", 10);
    this->joint3_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint3_controller/command", 10);
    this->tip_position_subscriber = nh.subscribe("/gazebo/link_states", 10, &InverseKinematics::LinkStatesCb, this);

    // this->create_target_points();

    // this->spawn_sphere_to_gazebo();
}

void InverseKinematics::create_target_points(){

    std::srand(static_cast<unsigned int>(std::time(nullptr))); // Seed the random number generator
    while (target_vector.size() < static_cast<size_t>(number_of_target)) {
        x_position = (static_cast<double>(std::rand()) / RAND_MAX * 2 - 1) * radius;
        y_position = (static_cast<double>(std::rand()) / RAND_MAX * 2 - 1) * radius;
        z_position = center.z + (static_cast<double>(std::rand()) / RAND_MAX * 2 - 1) * radius;

        // Check if the point is inside the sphere
        if (std::pow(x_position, 2) + std::pow(y_position, 2) + std::pow(z_position - center.z, 2) <= std::pow(radius, 2)) {
            target_vector.push_back({x_position, y_position, z_position});
        }
    }
    target_vector.push_back(home_pose);

}
void InverseKinematics::spawn_sphere_to_gazebo() {
    if (!target_vector.empty()) {
        // Since the last elemnet is for home position, don't spawn a sphere
        for (auto it = target_vector.begin(); it != target_vector.end() - 1; ++it) {
            const auto& point = *it;
            sphere_position.position.x = point.x;
            sphere_position.position.y = point.y;
            sphere_position.position.z = point.z;
            sphereSpawner.spawnSphereToGazebo(sphere_position);

            ROS_INFO("Spawning the sphere at the position: %f %f %f", point.x, point.y, point.z);
        }
    }
}

void InverseKinematics::LinkStatesCb(const gazebo_msgs::LinkStatesConstPtr &msg) {
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == tip) {
            tip_position = msg->pose[i].position;
        }
    }
}

void InverseKinematics::move_robot(){
    // x = 0.5*[sin(teta1) * sin(teta2) * cos(teta3) + sin(teta1) * cos(teta2) * sin(teta3)] + 0.5 * sin(teta1) * sin(teta2)
    // y = -0.5*[cos(teta1) * sin(teta2) * cos(teta3) + cos(teta1) * cos(teta2) * sin(teta3)] - 0.5 * cos(teta1) * sin(teta2)
    // z = 0.5*[cos(teta2) * cos(teta3) - sin(teta2) * sin(teta3)] + 0.5*cos(teta2) + 1
    // solve above equations and find teta1, teta2 and teta3 values
    ros::Rate rate(ros_rate);
    int count = 0;
    for (const auto& point : target_vector) {

        joint1_angle = atan2(point.x , point.y);
        joint1_angle *= -1;
        double t_1 = point.x / link_length / sin(joint1_angle);
        double t_2 = (point.z - 1) / link_length;
        joint2_angle = atan2(t_1, t_2) - 0.5 * acos((pow(t_1, 2) + pow(t_2, 2) - 2) / 2);
        joint3_angle = acos((pow(t_1, 2) + pow(t_2, 2) - 2) / 2);

        if(joint2_angle < 0){ joint2_angle += 2 * M_PI;}
        if(joint3_angle < 0){ joint3_angle += 2 * M_PI;}

        // joint3_angle = acos((pow((2*point.z - 2),2) / 2) + 2 * (pow(point.x, 2) + pow(point.y, 2)));
        // if(joint3_angle < 0) joint3_angle += 6.2831853;
        // else if(joint3_angle > 6.2831853) joint3_angle -= 6.2831853;
        // joint2_angle = atan2((2 * sqrt(pow(point.x, 2) + pow(point.y, 2))) , (2 * point.z - 2)) - (joint3_angle / 2);

        ROS_INFO("Joint values: %f %f %f", joint1_angle, joint2_angle, joint3_angle);

        joint1_command_value.data = joint1_angle;
        joint2_command_value.data = joint2_angle;
        joint3_command_value.data = joint3_angle;

        ROS_INFO("Target pose: %f %f %f", point.x, point.y, point.z);
        ROS_INFO("Tip position: %f %f %f", tip_position.x, tip_position.y, tip_position.z);

        while (ros::ok()) {
            changeColorPose.position.x = point.x;
            changeColorPose.position.y = point.y;
            changeColorPose.position.z = point.z;
            joint1_publisher.publish(joint1_command_value);
            joint2_publisher.publish(joint2_command_value);
            joint3_publisher.publish(joint3_command_value);
            if(isTouching(point)){
                if(count == number_of_target){
                    ROS_INFO("GOING TO HOME POSITION");
                    break;
                }
                ROS_INFO("Touched to the sphere of index: %d.", count);
                sphereSpawner.changeSphereColor(changeColorPose, "sphere" + std::to_string(count));
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }
        count++;

    }
    ROS_INFO("MISSION FINISHED");
}


// Calculate distance between tip position and target point
double InverseKinematics::calculateDistance(const sphere_pose& a) {
    return std::sqrt(std::pow(tip_position.x - a.x, 2) + std::pow(tip_position.y - a.y, 2) + std::pow(tip_position.z - a.z, 2));
}

bool InverseKinematics::isTouching(const sphere_pose& point) {
    return calculateDistance(point) <= threshold;
}