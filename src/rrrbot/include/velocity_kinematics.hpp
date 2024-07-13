/*
Date:29/11/2023
Developed by: Musa Almaz
Project: Velocity Kinematics
Summary: This header file is for doing velocity kinematic tasks.
*/ 

#ifndef VELOCITY_KINEMATICS_HPP
#define VELOCITY_KINEMATICS_HPP

#include <cmath>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/LinkStates.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <matplotlibcpp.h>
#include <vector>

class VelocityKinematics{
    public:
        /*
        Date:29/11/2023
        Developed by: Musa Almaz
        Summary: Print and write Jacobian matrix to file 
        Input:   No explicit input, uses calculated Jacobian matrix
        Output:  No explicit output.
        */
        void printWriteJacobian();

        /*
        Date:29/11/2023
        Developed by: Musa Almaz
        Summary: Publish necessary joint angles to move robot
        Input:   No explicit input.
        Output:  No explicit output.
        Additional info: Calls printWriteJacobian, computeJacobian, updateKinematicMap functions
        */
        void applyVelocity();

        /*
        Date:29/11/2023
        Developed by: Musa Almaz
        Summary: Updates kinematic map with change in position and orientation.
        Input:   No explicit input.
        Output:  No explicit output.
        Additional info: Uses multiplyMatrices function for matrix multiplication. 
        Uses previous kinematic map, Translation change and rotational change matrices.
        */
        void updateKinematicMap();

        /*
        Date:29/11/2023
        Developed by: Musa Almaz
        Summary: Prints the final kinematic map
        Input:   No explicit input.
        Output:  No explicit output.
        Additional info: Final kinematic map is computed in updateKinematicMap function.
        */
        void printFinalKinematicMap();

        /*
        Date:29/11/2023
        Developed by: Musa Almaz
        Summary: Prints the position and orientation change
        Input:   No explicit input.
        Output:  No explicit output.
        Additional info: Position change is linear velocity times delta(t) and orientation change is angular velocity times delta(t).
        */
        void printPoseOrientationChange();

        /*
        Date:29/11/2023
        Developed by: Musa Almaz
        Summary: Constructor for the VelocityKinematics class.
        Input:   No explicit input.
        Output:  No explicit output.
        Additional info: Reads the Jacobian, angular and linear velocities from the related files and computes the derivative of angles.
        J is Jacobian matrix, K is the linaer and angulr velocity, derivative of q is calculated as: d(q)/dt = (JT * J)^-1 * JT * K
        */
        void calculateInverseKinematic();

        /*
        Date:29/11/2023
        Developed by: Musa Almaz
        Summary: Constructor for the VelocityKinematics class.
        Input:   No explicit input.
        Output:  No explicit output.
        */
        VelocityKinematics(const ros::NodeHandle& nh_);

        ros::NodeHandle nh;


    private:
        /*
        Date:29/11/2023
        Developed by: Musa Almaz
        Summary: Computes the Jacobian matrix, linear and angular velocity
        Input:   No explicit input.
        Output:  No explicit output.
        */
        void computeJacobian();
        
        /*
        Date:29/11/2023
        Developed by: Musa Almaz
        Summary: Multiplies the a and b matrices, and writes the result to result matrix.
        Input:   a, b and result matrices with size 4x4
        Output:  No explicit output.
        */
        void multiplyMatrices(const double a[4][4], const double b[4][4], double result[4][4]); 
        
        double link_length = 0.5; // as meters, assume l2 = l3
        double theta1 = 0;
        double theta2 = 0;
        double theta3 = 0;
        double d_theta1;
        double d_theta2;
        double d_theta3;

        // Declear the necessary matrices
        double Jacobian[6][3];
        double linearVel[3];
        double angularVel[3];
        double Ksi[6];

        std::string jacobianFileName = "jacobian.txt";
        std::string linearVelFileName = "linearVel.txt";
        std::string angularVelFileName = "angularVel.txt";
        std::fstream JacobianFile;
        std::fstream linearVelFile;
        std::fstream angularVelFile;

        double step_size = 0.01; // delta time
        int max_step = 600;
        int step = 0;
        int numRowsJacobian = 6; // in this case, Jacobian is 6x3
        int numColsJacobian = 3;

        int ros_rate = 10; // initialize the ros rate

        // define the publishers
        ros::Publisher joint1_publisher;
        ros::Publisher joint2_publisher;
        ros::Publisher joint3_publisher;
        std_msgs::Float64 joint1_command_value, joint2_command_value, joint3_command_value;

        // Declare and initialize the initial kinematic map
        double initialKinematicMap[4][4] = {{0, 0, 1, 0}, {0, -1, 0, 0}, {1, 0, 0, 2}, {0, 0, 0, 1}};
        double T_d[4][4];
        double R_t[4][4];
        double finalKinematicMap[4][4];
        double tempMatrix[4][4]; // Temporary matrix to hold intermediate results

        double calculated_d_theta1[600];
        double calculated_d_theta2[600];
        double calculated_d_theta3[600];

        std::vector<double> given_d_theta1, given_d_theta2, given_d_theta3;

};









#endif // VELOCITY_KINEMATICS_HPP