/*
Date:19/12/2023
Developed by: Musa Almaz
Project: Path Planning using APF
Summary: This source file is for the robot_controller.hpp
This includes the necessary function definitions for the tasks.
*/ 
#include "robot_controller.hpp"

RobotController::RobotController(const ros::NodeHandle& nh_) : nh (nh_){
    this->robot_position_subscriber = nh.subscribe("/gazebo/model_states", 1, &RobotController::ModelStatesCb, this);
    this->command_velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    file_reader();
    spawn_obstacles_in_gazebo();

}
void RobotController::file_reader() {

    std::ifstream file(obstacle_file);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << obstacle_file << std::endl;
        return;
    }

    std::string line;
    if (std::getline(file, line)) { // Read the first line for the goal position
        std::istringstream iss(line);
        if (!(iss >> goal_position[0] >> goal_position[1])) {
            std::cerr << "Error reading goal position" << std::endl;
            return;
        }
    }

    while (std::getline(file, line)) { // Read the remaining lines for obstacles
        std::istringstream iss(line);
        Obstacle obstacle;
        if (iss >> obstacle.pos_x >> obstacle.pos_y >> obstacle.radius >> obstacle.height) {
            obstacle_info.push_back(obstacle);
        } else {
            std::cerr << "Error reading an obstacle line" << std::endl;
        }
        obstacle_number++;
    }

    file.close();
}

void RobotController::move_robot(){
    ros::Rate control_rate(ROS_RATE);
    int counter = 0;
    while(ros::ok()){
        if(counter == 20){
            std::array<double,2> vec = velocity_APF();
            double linear_x_vel = sqrt(pow(vec[0], 2) + pow(vec[1], 2));

            if(max_lin_vel < linear_x_vel){
                linear_x_vel = max_lin_vel;
            }else if(linear_x_vel < min_lin_vel){
                linear_x_vel = min_lin_vel;
            
            }
            double theta_angle =  atan2(vec[1], vec[0]);
            double heading = atan2(2.0 * (robot_orientation.w * robot_orientation.z + robot_orientation.x * robot_orientation.y), 1.0 -
             2.0 * (robot_orientation.y * robot_orientation.y + robot_orientation.z * robot_orientation.z));
            std::cout<<"heading" << heading << "\n";
            // Check the limits for angular velocity
            double angle_diff = theta_angle - heading; // + M_PI / 2.0;

            if (angle_diff > max_angular_vel) {
                angle_diff = max_angular_vel;
            } else if (angle_diff < min_angular_vel) {
                angle_diff = min_angular_vel;
            }

            command_vel.linear.x = linear_x_vel;
            command_vel.angular.z = angle_diff;
            std::cout<<"Position: " << current_robot_position.x <<" " << current_robot_position.y <<"\n";
            ROS_INFO("Published velocities : %f, %f", command_vel.linear.x, command_vel.angular.z);
            counter = 0;
        }
        command_velocity_publisher.publish(command_vel);
        ros::spinOnce();
        control_rate.sleep();
        counter++;
        if(sqrt(pow(current_robot_position.x - goal_position[0], 2) + pow(current_robot_position.y - goal_position[1], 2)) < threshold){
            command_vel.linear.x = 0;
            command_vel.angular.z = 0;
            command_velocity_publisher.publish(command_vel);
            ros::spinOnce();
            control_rate.sleep();
            break;
        }
    }
    ROS_INFO("Reached the goal");
}
void RobotController::ModelStatesCb(const gazebo_msgs::ModelStatesConstPtr &msg) {
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "robot") {
            current_robot_position = msg->pose[i].position;
            robot_orientation = msg->pose[i].orientation;
        }
    }
}

std::array<double,2> RobotController::velocity_APF(){
    std::array<double,2> apfVector;
    std::array<double,2> attVector = {0.0, 0.0};
    std::array<double,2> repVector = {0.0, 0.0};
    // Attractive force towards the goal
    attVector[0] += ATTRACTIVE_GAIN * (goal_position[0] - current_robot_position.x);
    attVector[1] += ATTRACTIVE_GAIN * (goal_position[1] - current_robot_position.y);

    // Repulsive force from obstacles
    for (const auto& obstacle : obstacle_info) {
        double B_i = pow(current_robot_position.x - obstacle.pos_x, 2) + pow(current_robot_position.y - obstacle.pos_y, 2) - pow(robot_radius + obstacle.radius, 2);
        repVector[0] += (current_robot_position.x - obstacle.pos_x) / B_i;
        repVector[1] += (current_robot_position.y - obstacle.pos_y) / B_i;
    }
    double gamma = pow(current_robot_position.x - goal_position[0], 2) + pow(current_robot_position.y - goal_position[1], 2);
    repVector[0] *= gamma * REPULSIVE_GAIN;
    repVector[1] *= gamma * REPULSIVE_GAIN;

    apfVector[0] = attVector[0] + repVector[0];
    apfVector[1] = attVector[1] + repVector[1];

    return apfVector;

}

void RobotController::print(){
    ROS_INFO("Robot position : %f, %f", current_robot_position.x, current_robot_position.y);
    std::cout<<"Goal position" << goal_position[0] <<" " << goal_position[1] <<"\n";
}

std::string RobotController::create_cylinder_SDF(const Obstacle& obstacle, int id) {
    std::ostringstream sdf;
    sdf << "<sdf version='1.6'>"
        << "<model name='obstacle_" << id << "'>"
        << "<static>true</static>"
        // Omitting the pose here, as it will be set during the spawning process
        << "<link name='link'>"
        << "<collision name='collision'>"
        << "<geometry>"
        << "<cylinder><radius>" << obstacle.radius << "</radius><length>" << obstacle.height << "</length></cylinder>"
        << "</geometry>"
        << "</collision>"
        << "<visual name='visual'>"
        << "<geometry>"
        << "<cylinder><radius>" << obstacle.radius << "</radius><length>" << obstacle.height << "</length></cylinder>"
        << "</geometry>"
        << "<material>"
        << "<ambient>1 0 0 1</ambient>" // Red color
        << "<diffuse>1 0 0 1</diffuse>" // Red color
        << "<specular>1 0 0 1</specular>" // Red color
        << "</material>"
        << "</visual>"
        << "</link>"
        << "</model>"
        << "</sdf>";
    return sdf.str();
}

void RobotController::spawn_obstacles_in_gazebo() {
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel spawn_model;

    for (size_t i = 0; i < obstacle_info.size(); ++i) {
        spawn_model.request.model_xml = create_cylinder_SDF(obstacle_info[i], i);
        spawn_model.request.model_name = "obstacle_" + std::to_string(i);
        geometry_msgs::Pose obstaclePose;
        obstaclePose.position.x = obstacle_info[i].pos_x;
        obstaclePose.position.y = obstacle_info[i].pos_y;
        spawn_model.request.initial_pose = obstaclePose;

        if (!client.call(spawn_model)) {
            ROS_ERROR("Failed to spawn model %s", spawn_model.request.model_name.c_str());
        }
    }
}