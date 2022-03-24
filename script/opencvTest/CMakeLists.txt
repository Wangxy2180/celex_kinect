cmake_minimum_required(VERSION 3.0.2)
project(testor)
find_package(OpenCV REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS
cv_bridge)

find_package(Eigen3)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}

)

add_executable(${PROJECT_NAME}_exe ./opencv_test.cpp)

target_link_libraries(${PROJECT_NAME}_exe
    ${catkin_INCLUDE_DIRS}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
  ${Eigen_LIBRARIES}

)
