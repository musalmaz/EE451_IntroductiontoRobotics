/*
Date:29/11/2023
Developed by: Musa Almaz
Project: Velocity Kinematics
Summary: This source file is for the velocity_kinematics.hpp
This includes the necessary function definitions for the tasks.
*/ 

#include "velocity_kinematics.hpp"

VelocityKinematics::VelocityKinematics(const ros::NodeHandle& nh_) : nh(nh_){
    this->joint1_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint1_controller/command", 10);
    this->joint2_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint2_controller/command", 10);
    this->joint3_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint3_controller/command", 10);

}

// Use δo(t) and δϕ(t) to incrementally update the kinematic map A(q(t)).
void VelocityKinematics::updateKinematicMap(){
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (i == j && i != 3) {
                T_d[i][j] = 1;
            } else if (j == 3 && i != 3) {
                T_d[i][j] = linearVel[i] * step_size;
            } else {
                T_d[i][j] = 0;
            }
        }
    }
    T_d[3][3] = 1;

    R_t[0][0] = R_t[1][1] = R_t[2][2] = R_t[3][3] = 1;
    R_t[0][3] = R_t[1][3] = R_t[2][3] = R_t[3][0] = R_t[3][1] = R_t[3][2] = 0;

    R_t[0][2] = angularVel[1] * step_size;
    R_t[2][0] = -1 * angularVel[1] * step_size;

    R_t[0][1] = -1 * angularVel[2] * step_size;
    R_t[1][0] = angularVel[2] * step_size;

    R_t[1][2] = -1 * angularVel[0] * step_size;
    R_t[2][1] = angularVel[2] * step_size;

    // First, multiply T_d and R_t
    multiplyMatrices(T_d, R_t, tempMatrix);

    // Then, multiply the result with initialKinematicMap
    multiplyMatrices(tempMatrix, initialKinematicMap, finalKinematicMap);

    // Hold final kinematic map on initial kinematic map

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            initialKinematicMap[i][j] = finalKinematicMap[i][j];
        }
    }

}

void VelocityKinematics::printFinalKinematicMap(){
    std::cout << "Printing final(updated) kinematic map \n";
    for(int i = 0; i < 4; i++){
        for(int j = 0; j< 4; j++){
            std::cout << finalKinematicMap[i][j] << " ";
        }
        std:: cout << "\n";
    }
}

void VelocityKinematics::multiplyMatrices(const double a[4][4], const double b[4][4], double result[4][4]){
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}    

// Print the Jacobian matrix J(q(t)), v(t) and ω(t)   on the screen. Save
// these values for later use
void VelocityKinematics::printWriteJacobian(){
    std::cout << "Printing Jacobian:\n";
    for(int i = 0; i < 6; i++){
        for(int j  = 0; j < 3; j++){
            std::cout << Jacobian[i][j] << " "; 
        }
        std::cout << "\n";
    }
    std::cout << "\n";
    JacobianFile.open(jacobianFileName,std::ios::out | std::ios::app);
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) {
            JacobianFile << std::setw(10) << Jacobian[i][j] << " ";
        }
        JacobianFile << "\n";
    }
    JacobianFile.close();

    std::cout << "Printing linear velocity:\n";
    for(int i = 0; i< 3; i++){
        std::cout << linearVel[i] << " ";
    }
    std::cout << "\n";
    linearVelFile.open(linearVelFileName, std::ios::out | std::ios::app);
    for(int i = 0; i< 3; i++){
        linearVelFile << linearVel[i] << " ";
    }
    linearVelFile << "\n";
    linearVelFile.close();

    std::cout << "Printing angular velocity:\n";
    for(int i = 0; i< 3; i++){
        std::cout << angularVel[i] << " ";
    }
    std::cout << "\n";
    angularVelFile.open(angularVelFileName, std::ios::out | std::ios::app);
    for(int i = 0; i< 3; i++){
        angularVelFile << angularVel[i] << " ";
    }
    angularVelFile << "\n";
    angularVelFile.close();

}
 // Use the linear and angular velocities to determine the resulting change in position δo(t) ∈ R 3
// and orientation δϕ(t) ∈ SO(3)
void VelocityKinematics::printPoseOrientationChange(){
    std::cout << "Orientation change: \n";
    std::cout << angularVel[0] * step_size << " " << angularVel[1] * step_size << " " << angularVel[2] * step_size <<"\n";

    std::cout << "Position change: \n";
    std::cout << linearVel[0] * step_size << " " << linearVel[1] * step_size << " " << linearVel[2] * step_size <<"\n";
}
// First compute the Jacobian matrix J(q(t)), v(t) and ω(t).
void VelocityKinematics::computeJacobian(){
    // Compute the Jacobian matrix
    Jacobian[0][0] = cos(theta1) * link_length * (sin(theta2 + theta3) + sin(theta2));
    Jacobian[0][1] = sin(theta1) * link_length * (cos(theta2 + theta3) + cos(theta2));
    Jacobian[0][2] = link_length * sin(theta1) * cos(theta2 + theta3);
    Jacobian[1][0] = link_length * sin(theta1) * (sin(theta2 + theta3) + sin(theta2));
    Jacobian[1][1] = -1 * link_length * cos(theta1) * (cos(theta2 + theta3) + cos(theta2));
    Jacobian[1][2] = -1 * link_length * cos(theta1) * cos(theta2 + theta3);
    Jacobian[2][0] = 0;
    Jacobian[2][1] = -1 * link_length * (sin(theta2 - theta3) + sin(theta2));
    Jacobian[2][2] = -1 * link_length * sin(theta2 + theta3);
    Jacobian[3][0] = 0;
    Jacobian[3][1] = cos(theta1);
    Jacobian[3][2] = cos(theta2);
    Jacobian[4][0] = 0;
    Jacobian[4][1] = sin(theta2);
    Jacobian[4][2] = sin(theta1);
    Jacobian[5][0] = 1;
    Jacobian[5][1] = 0;
    Jacobian[5][2] = 0;

    // Compute the angular velocity
    angularVel[0] = cos(theta1) * (d_theta2 + d_theta3);
    angularVel[1] = sin(theta1) * (d_theta2 + d_theta3);
    angularVel[2] = d_theta3;

    // Compute the linear velocity
    linearVel[0] = Jacobian[0][0] * d_theta1 + Jacobian[0][1] * d_theta2 + Jacobian[0][2] * d_theta3;
    linearVel[1] = Jacobian[1][0] * d_theta1 + Jacobian[1][1] * d_theta2 + Jacobian[1][2] * d_theta3;
    linearVel[2] = Jacobian[2][0] * d_theta1 + Jacobian[2][1] * d_theta2 + Jacobian[2][2] * d_theta3;

}

void VelocityKinematics::applyVelocity(){
    ros::Rate rate(ros_rate);
    while(step < max_step && ros::ok()){

        if(step <= 100){
            d_theta1 = 0.1 * (step * step_size);
            theta1 += d_theta1 * step_size;   // Update theta1

            d_theta3 = 0.1 * (step * step_size);
            theta3 += d_theta3 * step_size; // update theta3

            d_theta2 = 0.01 * (step * step_size);
            theta2 += d_theta2 * step_size; // update theta2
        }
        else if(100 < step < 500){
            d_theta1 = 0.3;
            theta1 += d_theta1 * step_size;   // Update theta1

            d_theta3 = 0.3;
            theta3 += d_theta3 * step_size; // update theta3

            d_theta2 = 0.01;
            theta2 += d_theta2 * step_size; // update theta2
        }
        else{
            d_theta1 = 0.3 - 0.1 * (step * step_size - 5);
            theta1 += d_theta1 * step_size;   // Update theta1

            d_theta3 = 0.3 - 0.1 * (step * step_size - 5);
            theta3 += d_theta3 * step_size;   // Update theta3

            d_theta2 = 0.1 - 0.01 * (step * step_size - 5);
            theta2 += d_theta2 * step_size; // update theta2
        }
        computeJacobian();
        printWriteJacobian();
        printPoseOrientationChange();
        updateKinematicMap();
        printFinalKinematicMap();

        step++;

        joint1_command_value.data = theta1;
        joint2_command_value.data = theta2;
        joint3_command_value.data = theta3;

        joint1_publisher.publish(joint1_command_value);
        joint2_publisher.publish(joint2_command_value);
        joint3_publisher.publish(joint3_command_value);

        ros::spinOnce();
        rate.sleep();

    }
    ROS_INFO("FINISHED THE TASK.");
}

// Now consider applying inverse velocity kinematics. Using v(t) and ω(t) values found with J(q(t))
// for t = kδt for each k = 1, . . . , 600, try to find θ̇ 1 , θ̇ 2 , θ̇ 3 . Compare the results with those of Eq. 1 through
// generating a plot for each joint speed.
void VelocityKinematics::calculateInverseKinematic(){
    JacobianFile.open(jacobianFileName,std::ios::out | std::ios::app);
    if (!JacobianFile.is_open()) {
        std::cerr << "Failed to open Jacobian file, maybe path is wrong \n";
    }
    linearVelFile.open(linearVelFileName, std::ios::out | std::ios::app);
    if (!linearVelFile.is_open()) {
        std::cerr << "Failed to open LinearVel file, maybe path is wrong \n";
    }
    angularVelFile.open(angularVelFileName, std::ios::out | std::ios::app);
    if (!angularVelFile.is_open()) {
        std::cerr << "Failed to open AngularVel file, maybe path is wrong \n";
    }
    // Read files and calculate derivative of angles
    for(int k = 0; k < max_step; k++){
        for (int i = 0; i < numRowsJacobian; ++i) {
            for (int j = 0; j < numColsJacobian; ++j) {
                JacobianFile >> Jacobian[i][j];
            }
        }

        for(int i = 0; i < numColsJacobian; i++){
            linearVelFile >> linearVel[i];
        }

        for(int i = 0; i < numColsJacobian; i++){
            angularVelFile >> angularVel[i];
        }

        // Calculate Ksi from linear and angular velocities
        for(int i = 0; i< 3; i++){
            Ksi[i] = linearVel[i];
        }
        for(int i = 0; i< 3; i++){
            Ksi[i + 3] = angularVel[i];
        }
        // Convert the C array to Eigen::MatrixXd
        Eigen::Map<Eigen::MatrixXd> J_Eigen(&Jacobian[0][0], numRowsJacobian, numColsJacobian);
        Eigen::Map<Eigen::VectorXd> Ksi_Vector(&Ksi[0], numRowsJacobian); // For a column vector

        // Now you can use Eigen's matrix operations
        Eigen::MatrixXd JTJ = J_Eigen.transpose() * J_Eigen;

        // Invert J^T * J
        Eigen::MatrixXd JTJ_inv = JTJ.inverse();

        // Calculate the final result: (J^T * J)^-1 * J^T * K
        Eigen::VectorXd result = JTJ_inv * J_Eigen.transpose() * Ksi_Vector;

        calculated_d_theta1[k] = result[0];
        calculated_d_theta2[k] = result[1];
        calculated_d_theta3[k] = result[2];

    }
    // Calculate derivative of angles with given conitions on 600 step
    // Calculate t1, t2, and t3 with the specified conditions and step size
    for (double t = 0.0; t < 6.0; t += step_size) {
        double t1, t2, t3;

        // Apply the conditions for t1
        if (t <= 1.0) {
            t1 = 0.1 * t;
        } else if (t < 5.0) {
            t1 = 0.3;
        } else if(t > 5){
            t1 = 0.3 - 0.1 * (t - 5.0);
        }

        // Apply the conditions for t2
        if (t <= 1.0) {
            t2 = 0.01 * t;
        } else if (t < 5.0) {
            t2 = 0.1;
        } else if(t > 5){
            t2 = 0.1 - 0.01 * (t - 5.0);
        }

        // Apply the conditions for t3
        if (t <= 1.0) {
            t3 = 0.1 * t;
        } else if (t < 5.0) {
            t3 = 0.3;
        } else if(t > 5){
            t3 = 0.3 - 0.1 * (t - 5.0);
        }

        // Add the values to the arrays
        given_d_theta1.push_back(t1);
        given_d_theta2.push_back(t2);
        given_d_theta3.push_back(t3);
    }
    given_d_theta1.pop_back();
    given_d_theta2.pop_back();
    given_d_theta3.pop_back();

    // Generate x values (0, 1, 2, ..., 599)
    std::vector<double> x(600);
    for (int i = 0; i < 600; ++i) {
        x[i] = i;
    }

    std::vector<double> calculated_d_theta1_vector(calculated_d_theta1, calculated_d_theta1 + max_step);
    std::vector<double> calculated_d_theta2_vector(calculated_d_theta2, calculated_d_theta2 + max_step);
    std::vector<double> calculated_d_theta3_vector(calculated_d_theta3, calculated_d_theta3 + max_step);

    // Plot for theta1
    matplotlibcpp::figure(); // Create a new figure
    matplotlibcpp::plot(x, calculated_d_theta1_vector, "r-");
    matplotlibcpp::plot(x, given_d_theta1, "g-");
    matplotlibcpp::xlabel("Time * 100");
    matplotlibcpp::ylabel("Values");
    matplotlibcpp::title("Plot of calculated_d_theta1 and given_d_theta1");
    matplotlibcpp::legend();
    matplotlibcpp::save("theta1.png");
    matplotlibcpp::show();

    // Plot 2
    matplotlibcpp::figure(); // Create a new figure
    matplotlibcpp::plot(x, calculated_d_theta2_vector, "r-");
    matplotlibcpp::plot(x, given_d_theta2, "g-");
    matplotlibcpp::xlabel("Time * 100");
    matplotlibcpp::ylabel("Values");
    matplotlibcpp::title("Plot of calculated_d_theta2 and given_d_theta2");
    matplotlibcpp::legend();
    matplotlibcpp::save("theta2.png");
    matplotlibcpp::show();

    // Plot 3
    matplotlibcpp::figure(); // Create a new figure
    matplotlibcpp::plot(x, calculated_d_theta3_vector, "r-");
    matplotlibcpp::plot(x, given_d_theta3, "g-");
    matplotlibcpp::xlabel("Time * 100");
    matplotlibcpp::ylabel("Values");
    matplotlibcpp::title("Plot of calculated_d_theta3 and given_d_theta3");
    matplotlibcpp::legend();
    matplotlibcpp::save("theta3.png");
    matplotlibcpp::show();

}