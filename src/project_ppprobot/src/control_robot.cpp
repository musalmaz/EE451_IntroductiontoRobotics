#include <project_ppprobot/control_robot.hpp>

Control_Robot::Control_Robot(const ros::NodeHandle& nh_) : nh(nh_){
    this->z_joint_pub = nh.advertise<std_msgs::Float64>("/PPP_Robot/z_prismatic_joint_position_controller/command", 1);
    this->y_joint_pub = nh.advertise<std_msgs::Float64>("/PPP_Robot/y_prismatic_joint_position_controller/command", 1);
    this->x_joint_pub = nh.advertise<std_msgs::Float64>("/PPP_Robot/x_prismatic_joint_position_controller/command", 1);
}

void Control_Robot::cubePoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    cubePosition.position.x = msg->position.x;
    cubePosition.position.y = msg->position.y;
    cubePosition.position.z = msg->position.z; 
    // Copy the position of the cube
    copy_cubePosition = cubePosition;
    callback_received = true; 
}

void Control_Robot::get_cube_pose(){
    callback_received = false;
    this->sub_cube_pose = nh.subscribe<geometry_msgs::Pose>("cube_pose", 10, &Control_Robot::cubePoseCallback, this);
    // Wait for the callback to be triggered
    ros::Rate rate(ros_rate); 
    while (!callback_received && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_ONCE("Cube pose : %f %f %f", cubePosition.position.x, cubePosition.position.y, cubePosition.position.z);
}

void Control_Robot::move_forward_x(){
    // Wait for the callback to be triggered
    ros::Rate rate(ros_rate); 
    while (!callback_received && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    // Calculate the x and y distance needed to traverse
    x_distance_to_cube = copy_cubePosition.position.x - initial_x_pose - cube_half_edge - tolerance;
    y_distance_to_cube = copy_cubePosition.position.y - initial_y_pose - cube_half_edge;

    ROS_INFO("The neeeded distance in x any y : %f %f", x_distance_to_cube, y_distance_to_cube);

    ROS_INFO("Moving x and y direction ");
    // Move along x axis
    std_msgs::Float64 x_msg;
    x_msg.data = x_distance_to_cube;
    std_msgs::Float64 y_msg;
    y_msg.data = y_distance_to_cube;
    for(int i = 0; i< 30; i++){ 
        y_joint_pub.publish(y_msg);
        x_joint_pub.publish(x_msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Moving z direction (down)");
    // Move along z axis (down)
    std_msgs::Float64 z_msg;
    z_msg.data = down_along_z;
    for(int i = 0; i< 30; i++){      
        z_joint_pub.publish(z_msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Moving x direction (forward)");
    // Move along x axis (forward)
    std_msgs::Float64 x_msg_forward;
    x_msg_forward.data = (translation_distance + tolerance);
    for(int i = 0; i< 30; i++){
        x_joint_pub.publish(x_msg_forward);
        ros::spinOnce();
        rate.sleep();
    }

}

void Control_Robot::move_backward_x(){
    // Wait for the callback to be triggered
    ros::Rate rate(ros_rate); 
    while (!callback_received && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    // Calculate the x and y distance needed to traverse
    x_distance_to_cube = copy_cubePosition.position.x - initial_x_pose + cube_half_edge + tolerance;
    y_distance_to_cube = copy_cubePosition.position.y - initial_y_pose - cube_half_edge;

    ROS_INFO("The neeeded distance in x any y : %f %f", x_distance_to_cube, y_distance_to_cube);

    ROS_INFO("Moving x and y direction ");
    // Move along x axis
    std_msgs::Float64 x_msg;
    x_msg.data = x_distance_to_cube;
    std_msgs::Float64 y_msg;
    y_msg.data = y_distance_to_cube;
    for(int i = 0; i< 30; i++){ 
        y_joint_pub.publish(y_msg);
        x_joint_pub.publish(x_msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Moving z direction (down)");
    // Move along z axis (down)
    std_msgs::Float64 z_msg;
    z_msg.data = down_along_z;
    for(int i = 0; i< 30; i++){      
        z_joint_pub.publish(z_msg);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Moving x direction (backward)");
    // Move along x axis (forward)
    std_msgs::Float64 x_msg_backward;
    x_msg_backward.data = -(translation_distance + tolerance);
    for(int i = 0; i< 30; i++){
        x_joint_pub.publish(x_msg_backward);
        ros::spinOnce();
        rate.sleep();
    }

}

void Control_Robot::move_forward_y(){
    // Wait for the callback to be triggered
    ros::Rate rate(ros_rate); 
    while (!callback_received && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    // Calculate the x and y distance needed to traverse
    x_distance_to_cube = copy_cubePosition.position.x - initial_x_pose;
    y_distance_to_cube = copy_cubePosition.position.y - initial_y_pose - cube_half_edge - tolerance;

    ROS_INFO("The neeeded distance in x any y : %f %f", x_distance_to_cube, y_distance_to_cube);

    ROS_INFO("Moving x and y direction ");

    // Move along x and y axis
    std_msgs::Float64 x_msg;
    x_msg.data = x_distance_to_cube;
    std_msgs::Float64 y_msg;
    y_msg.data = y_distance_to_cube;
    for(int i = 0; i< 30; i++){ 
        y_joint_pub.publish(y_msg);
        x_joint_pub.publish(x_msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Moving z direction (down)");
    // Move along z axis (down)
    std_msgs::Float64 z_msg;
    z_msg.data = down_along_z;
    for(int i = 0; i< 30; i++){      
        z_joint_pub.publish(z_msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Moving y direction (forward)");
    // Move along y axis (forward)
    std_msgs::Float64 y_msg_forward;
    y_msg_forward.data = (translation_distance + tolerance);
    for(int i = 0; i< 30; i++){
        y_joint_pub.publish(y_msg_forward);
        ros::spinOnce();
        rate.sleep();
    }

}

void Control_Robot::move_backward_y(){
    // Wait for the callback to be triggered
    ros::Rate rate(ros_rate); 
    while (!callback_received && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    // Calculate the x and y distance needed to traverse
    x_distance_to_cube = copy_cubePosition.position.x - initial_x_pose;
    y_distance_to_cube = copy_cubePosition.position.y - initial_y_pose + cube_half_edge + tolerance;

    ROS_INFO("The neeeded distance in x any y : %f %f", x_distance_to_cube, y_distance_to_cube);

    ROS_INFO("Moving x and y direction ");

    // Move along x and y axis
    std_msgs::Float64 x_msg;
    x_msg.data = x_distance_to_cube;
    std_msgs::Float64 y_msg;
    y_msg.data = y_distance_to_cube;
    for(int i = 0; i< 30; i++){ 
        y_joint_pub.publish(y_msg);
        x_joint_pub.publish(x_msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Moving z direction (down)");
    // Move along z axis (down)
    std_msgs::Float64 z_msg;
    z_msg.data = down_along_z;
    for(int i = 0; i< 30; i++){      
        z_joint_pub.publish(z_msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Moving y direction (backward)");
    // Move along y axis (backward)
    std_msgs::Float64 y_msg_backward;
    y_msg_backward.data = -(translation_distance + tolerance);
    for(int i = 0; i< 30; i++){
        y_joint_pub.publish(y_msg_backward);
        ros::spinOnce();
        rate.sleep();
    }

}

void Control_Robot::move_cube(){
    // Random number engine
    std::default_random_engine generator(seed);
    // Uniform distribution between 0 and 3
    std::uniform_int_distribution<int> distribution(0,3);

    // Generate random integer between 0 and 3
    random_integer = distribution(generator);
    // Call different functions to move the cube
    switch (random_integer)
    {
    case 0:
        ROS_INFO("Moving the cube forward in the x direction.");
        move_forward_x();
        break;
    case 1:
        ROS_INFO("Moving the cube backward in the x direction.");
        move_backward_x();
        break;
    case 2:
        ROS_INFO("Moving the cube forward in the y direction.");
        move_forward_y();
        break;
    case 3:
        ROS_INFO("Moving the cube backward in the y direction.");
        move_backward_y();
        break;
    default:
        break;
    }
}

