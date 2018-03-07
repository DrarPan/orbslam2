cmake_minimum_required(VERSION 2.8.3)
project(orbslam2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -std=c++11")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 2.4.8 QUIET) #ignore opencv version
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(Qt4 REQUIRED)
include_directories("/usr/include/qt4")
include_directories("/usr/include/qt4/Qt")

find_package(catkin REQUIRED roscpp roslib tf geometry_msgs cv_bridge message_filters image_transport)
find_package(OpenGL REQUIRED)

include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

catkin_package()

add_library(ORB_SLAM2 SHARED
src/Camera.cc
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
)

add_library(ORB_SLAM2_PCL SHARED
src/Camera.cc
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/PointCloud/PointCloudMapping.cc
)

add_library(ORB_SLAM2_CAMPARAM src/Camera.cc)
target_link_libraries(ORB_SLAM2_CAMPARAM ${OpenCV_LIBS})

target_link_libraries(ORB_SLAM2
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

target_link_libraries(ORB_SLAM2_PCL
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${PCL_LIBRARIES}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
ORB_SLAM2
${catkin_LIBRARIES}
)

add_library(ARmodule src/ROS/AR/DataConverter.cc src/ROS/AR/MotionModel.cc src/ROS/AR/PlaneDetector.cc)
target_link_libraries(ARmodule ${OpenCV_LIBS} ORB_SLAM2_CAMPARAM ${catkin_LIBRARIES})

add_executable(test_qtviewer src/ROS/test_qtviewer.cc src/QtViewer.cc)
target_link_libraries(test_qtviewer Qt4::QtCore Qt4::QtGui Qt4::QtOpenGL ${Pangolin_LIBRARIES})

# components for AR
add_executable(test_ARmodule src/ROS/AR/ARModule.cc)
target_link_libraries(test_ARmodule ARmodule ${OpenCV_LIBS} ORB_SLAM2_CAMPARAM ${catkin_LIBRARIES})

# Node for monocular camera
add_executable(Mono src/ROS/ros_mono.cc)
target_link_libraries(Mono ${LIBS})

add_executable(MonoAR src/ROS/AR/ros_mono_ar.cc src/ROS/AR/ViewerAR.h src/ROS/AR/ViewerAR.cc)
target_link_libraries(MonoAR ${LIBS})

# Node for stereo camera
add_executable(Stereo src/ROS/ros_stereo.cc)
target_link_libraries(Stereo ${LIBS})

# Node for RGB-D camera
add_executable(RGBD src/ROS/ros_rgbd.cc)
target_link_libraries(RGBD ${LIBS})

add_executable(RGBDAR src/ROS/AR/ros_rgbd_ar.cc)
target_link_libraries(RGBDAR ARmodule  ${LIBS})

add_executable(ros_rgbd_map src/ROS/ros_rgbd_map.cc)
target_link_libraries(ros_rgbd_map ${LIBS})

add_executable(testLoadMap src/ROS/testLoadMap.cc)
target_link_libraries(testLoadMap ${LIBS})
