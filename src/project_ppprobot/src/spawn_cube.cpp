#include <project_ppprobot/spawn_cube.hpp>

Spawn_Cube::Spawn_Cube(const ros::NodeHandle& nh_) : nh(nh_){}

void Spawn_Cube::spawn_cube_to_gazebo(){

    time_seed = std::time(nullptr);
    seed = static_cast<unsigned int>(time_seed);
    std::srand(seed);
    // Generate the random values
    random_x = xMin + static_cast<double>(std::rand()) / RAND_MAX * (xMax - xMin);
    random_y = yMin + static_cast<double>(std::rand()) / RAND_MAX * (yMax - yMin);
    // define the position of the cube
    cubePose.position.x = random_x;
    cubePose.position.y = random_y;
    cubePose.position.z = z;

    // Open the URDF file
    std::ifstream urdfFile(urdfFilePath);

    // Check if the file is opened successfully
    if (!urdfFile) {
        ROS_ERROR("Failed to open URDF file: %s",urdfFilePath.c_str());
    }else{
        ROS_INFO("Urdf file opened.");
    }

    // Read the contents of the URDF file into a string
    std::string urdfContents((std::istreambuf_iterator<char>(urdfFile)),
                             std::istreambuf_iterator<char>());

    urdfFile.close();

    spawnModelRequest.request.initial_pose = cubePose;
    spawnModelRequest.request.model_xml = urdfContents;
    spawnModelRequest.request.model_name = "cube";
    
    spawnClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
     // Call the service to spawn the cube
    if (spawnClient.call(spawnModelRequest) && spawnModelRequest.response.success)
    {
        ROS_INFO("Spawned the cube successfully.");
    }
    else
    {
        ROS_ERROR("Failed to spawn model. Error message: %s", spawnModelRequest.response.status_message.c_str());
    }
}

void Spawn_Cube::cube_pose_publisher(){
    publishCubePose = nh.advertise<geometry_msgs::Pose>("cube_pose", 10);
    // Publish the position
    publishCubePose.publish(cubePose);

    ROS_INFO("Cube position : %f %f %f", cubePose.position.x, cubePose.position.y, cubePose.position.z);
}