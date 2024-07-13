#include <project_ppprobot/control_robot.hpp>

#define ROS_RATE 10

int main(int argc, char **argv){
    ros::init(argc, argv, "main_ppprobot");
    ros::NodeHandle nh;
    Control_Robot robot(nh);

    ros::Rate rate(ROS_RATE);
    // Connect the master and get the position of the cube
    for(int i = 0; i < 10; i++){
        robot.get_cube_pose();
        ros::spinOnce();
        rate.sleep();
    }
    // Translate the cube
    robot.move_cube();
    ros::spinOnce();
    rate.sleep();

    return 0;

}