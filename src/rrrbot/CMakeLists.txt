cmake_minimum_required(VERSION 3.0.2)
project(rrrbot)



find_package(catkin REQUIRED COMPONENTS
        roscpp
        gazebo_ros
        gazebo_ros_control
        gazebo_msgs
        controller_manager
        effort_controllers
        robot_state_publisher
        std_msgs
        geometry_msgs
        sensor_msgs
        )
        

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})



list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp gazebo_ros gazebo_ros_control controller_manager effort_controllers robot_state_publisher std_msgs sensor_msgs
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(random_kinematics src/random_kinematics.cpp include/kinematics.h src/kinematics.cpp)
target_link_libraries(random_kinematics ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(test_velocity_kinematics src/test_velocity_kinematics.cpp include/velocity_kinematics.hpp src/velocity_kinematics.cpp)
target_link_libraries(test_velocity_kinematics ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PYTHON_LIBRARIES})



add_executable(visualize_workspace src/visualize_workspace.cpp include/kinematics.h src/kinematics.cpp include/sphere_spawner.hpp src/sphere_spawner.cpp)
target_link_libraries(visualize_workspace ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(touch src/touch_sphere.cpp include/inverse_kinematics.hpp src/inverse_kinematics.cpp include/sphere_spawner.hpp src/sphere_spawner.cpp)
target_link_libraries(touch ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})