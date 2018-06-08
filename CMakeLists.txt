cmake_minimum_required(VERSION 2.8.0)
project(test_tsdf)


# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11")
#Flag for OpenMP
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/") #find external libraries
SET(CMAKE_BUILD_TYPE Release)
# Find library and package
## CUDA
find_package(CUDA 8.0 REQUIRED)
SET(CUDA_PROPAGATE_HOST_FLAGS OFF)
# Optional: Specify the architechture of your CUDA hardware here, for NVIDIA 1060
    #can be found in : https://developer.nvidia.com/cuda-gpus
SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3;-std=c++11 -D_MWAITXINTRIN_H_INCLUDED -lcudart -lcublas -lcurand -gencode arch=compute_61,code=sm_61)
find_package(OpenCV 3 REQUIRED PATHS "/home/jcl-stud/opencv3/")
MESSAGE("OpenCV version : ${OpenCV_VERSION}")
MESSAGE("OpenCV DIR : ${OpenCV_DIR}")
find_package(TinyXML REQUIRED)
#find_package(Boost 1.61 COMPONENTS system filesystem REQUIRED)
find_package(Boost REQUIRED)
MESSAGE("Boost version : ${Boost_VERSION}")
MESSAGE("Boost DIR : ${Boost_DIR}")
find_package(PCL 1.7.2 REQUIRED)
find_package(VTK REQUIRED)
MESSAGE("VTK version : ${VTK_VERSION}")
MESSAGE("VTK DIR : ${VTK_DIR}")
find_package(CURL REQUIRED)
find_library(Realsense_LIBS realsense2)
###########
## Build ##
###########
include_directories(

common/include
  ${OpenCV_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIR}
${realsense_INCLUDE_DIR}
${Boost_INCLUDE_DIR}
 ${CUDA_INCLUDE_DIRS}
)
file(GLOB TSDF_SRCS "${PROJECT_SOURCE_DIR}/common/src/cuda/*.cu")
CUDA_ADD_LIBRARY(tsdf_cuda STATIC "${TSDF_SRCS}")
target_link_libraries(tsdf_cuda ${OpenCV_LIBRARIES} )

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

#Show version

MESSAGE("PCL version : ${PCL_VERSION}")
MESSAGE("PCL DIR : ${PCL_DIR}")

#Realsense cameras
set(realsense_CPP_FILES
            common/src/realsense.cpp
            common/src/realsense2.cpp
    )
add_library(realsense ${realsense_CPP_FILES})
target_link_libraries(realsense
  ${OpenCV_LIBS}
${Realsense_LIBS}
${PCL_LIBRARIES}
)
#Dynamixel motor
set(dynamixel_CPP_Files common/src/dynamixel.c
                common/src/dxl_hal.c
                common/src/Dynamixel.cpp)
add_library(Dynamixel ${dynamixel_CPP_Files})


#Webserver
set(webserver_Files common/src/curlserver.cpp)
add_library(Webserver ${webserver_Files})
target_link_libraries(Webserver
${Boost_LIBRARIES}
${CURL_LIBRARIES}
)
#Common cpp files
set(project_CPP_FILES
	common/src/features.cpp
	common/src/filters.cpp
	common/src/registrator.cpp
	common/src/util.cpp
	common/src/segmentation.cpp
	common/src/aruco_marker.cpp	
        common/src/loader.cpp
        common/src/matrix_utils.cpp
        common/src/kinect_fusion.cpp
	)
add_library(project_library ${project_CPP_FILES})
target_link_libraries (project_library
  ${OpenCV_LIBS}
  ${TinyXML_LIBRARIES}
${PCL_LIBRARIES}
${Boost_LIBRARIES}
${Eigen_INCLUDE_DIRS}
${Realsense_LIBS}
)

#Pi tag files
#set(pi_tag_CPP_FILES
 # 	common/src/AbstractFiducialModel.cpp
#	common/src/FiducialModelPi.cpp
 # 	common/src/ellipserefine.cpp
#)


#add_library(pi_tag ${pi_tag_CPP_FILES})
#target_link_libraries(pi_tag
#  ${OpenCV_LIBS}
#  ${TinyXML_LIBRARIES}
#${PCL_LIBRARIES}
#${Boost_LIBRARIES}
#${Eigen_INCLUDE_DIRS}
#)
#set_target_properties(pi_tag PROPERTIES COMPILE_FLAGS "-D__LINUX__")
################################################################################

##Executable pi tag
#add_executable(pi_tag_test src/test_pitag.cpp ${project_CPP_FILES} )
#target_link_libraries(pi_tag_test pi_tag)
#set_target_properties(pi_tag_test  PROPERTIES COMPILE_FLAGS "-D__LINUX__")
##########################################################
# Executable for test_functions
add_executable(test_functions src/test_functions.cpp)
target_link_libraries( test_functions
realsense
Webserver
project_library
tsdf_cuda
)
##################################################
#Test reconstruction with charuco board given data in folder data
#add_executable(charuco_test src/test_charuco.cpp)
#target_link_libraries(charuco_test
 #   ${OpenCV_LIBS}
  #  ${TinyXML_LIBRARIES}
   #${PCL_LIBRARIES}
    #${Boost_LIBRARIES}
    #${Eigen_INCLUDE_DIRS}
 #${Realsense_LIBS}
#project_library
#)
##################################
#Executable for model
#add_executable(test_model src/test_model.cpp )
#target_link_libraries(test_model
 #   ${OpenCV_LIBS}
  #  ${TinyXML_LIBRARIES}
   # ${PCL_LIBRARIES}
   # ${Boost_LIBRARIES}
   # ${Eigen_INCLUDE_DIRS}
#realsense
#project_library
#)
####################################
#Executable for whole reconstruction with realsense and dynamixel
#add_executable(test_dynamixel_realsense src/test_dynamixel_realsense.cpp)
#target_link_libraries(test_dynamixel_realsense
#    project_library
#    Dynamixel
#    realsense
#)
###############################
#Executable for webserver asio test
#add_executable(test_webserver_asio src/test_webserver_asio.cpp)
#target_link_libraries(test_webserver_asio
#    Webserver
#    realsense
#    project_library

#)
###############################
#add_executable(realsense_dnn src/realsense_dnn.cpp)
#target_link_libraries(realsense_dnn
 #                       realsense
#)
################################
#Visualize realsense stream on pcl viewer
add_executable(test_realsense src/test_realsense.cpp)
target_link_libraries(test_realsense
                        realsense
                     project_library
                        Webserver
                        tsdf_cuda
)
#####################################
#Executable for webserver cURL test
#add_executable(multi_reconstruction src/multi_reconstruction.cpp)
#target_link_libraries(multi_reconstruction
   # Webserver
  #  realsense
 #   project_library
#)
#Executable for test_realsense_camera
#add_executable(test_object_contours src/test_object_contours.cpp)
#target_link_libraries(test_object_contours
 #   Webserver
  #  realsense
  #  project_library
#)
#Executable for final_implemetation.cpp
add_executable(reconstruction src/final_implemetation.cpp)
target_link_libraries(reconstruction
    Webserver
    realsense
    project_library
    tsdf_cuda
)
#Executable for data collection
#add_executable(collect_data src/data_collection.cpp)
#target_link_libraries(collect_data
#    Webserver
 #   realsense
 #   project_library
#)
#Executable for 2D segmentation
#add_executable(color_segmentation src/color_segmentation.cpp)
#target_link_libraries(color_segmentation
 #                       project_library)
#add_executable(color_segmentation_test src/color_segmentation_test.cpp)
#target_link_libraries(color_segmentation_test
 #                       project_library)

