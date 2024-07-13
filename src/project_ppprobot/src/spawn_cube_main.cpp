#include <project_ppprobot/spawn_cube.hpp>

#define ROS_RATE 20

int main(int argc, char **argv){
    ros::init(argc, argv, "spawn_cube");
    ros::NodeHandle nh;
    Spawn_Cube spawn(nh);
    // Connect the master
    ros::Rate rate(ROS_RATE);
    for(int i = 0; i < 10; i++){
        ros::spinOnce();
        rate.sleep();
    }
    // Spawn the cube to gazebo
    spawn.spawn_cube_to_gazebo();
    ros::spinOnce();
    // Publish the position of the cube
    while(true){
        spawn.cube_pose_publisher();
        ros::spinOnce();
        rate.sleep();
    }
}