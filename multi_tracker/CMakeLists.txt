#cmake_minimum_required (VERSION 2.6)
#project (Tutorial)

#FIND_PACKAGE(opencv2)
#find_package(opencv2 REQUIRED)
#include_directories(${OPENCV2_INCLUDE_DIRS})
#add_executable(imu_data_access_cmake_test imu_data_access.cpp)
#target_link_libraries(imu_data_access_cmake_test ${CURL_LIBRARIES})

#link_directories("/usr/local/include/opencv4/opencv2")
#find_package(OpenCV REQUIRED)
#include_directories(OpenCV_INCLUDE_DIR)
#find_package(OpenCV REQUIRED PATHS "/usr/local/include/opencv4/opencv2")



#find_package(OpenCV REQUIRED PATHS "/usr/local/include/opencv4/opencv2/" )#"/usr/include/opencv2/")
#find_package(OpenCV PATHS "/usr/local/include/opencv4/opencv2/" )
#include_directories(${OpenCV_INCLUDE_DIRS})
#target_link_libraries(OpenCV -L/home/jonathan/Documents/Git/realsense/Tracker/build)
#find_package(OpenCV_Ros REQUIRED PATHS "/usr/include/opencv2/")
#include_directories(${OpenCV_Ros_INCLUDE_DIRS})
#add_executable(tracker_test_to_run tracker_test.cpp)

cmake_minimum_required(VERSION 2.8)
project(multi_tracker_test_to_run)
find_package( OpenCV REQUIRED )
add_executable(multi_tracker_test_to_run tracking_multi.cpp)
target_link_libraries(multi_tracker_test_to_run ${OpenCV_LIBS} )