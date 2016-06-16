# Copyright: 2010 RobotCub Consortium
# Author: Sean Ryan Fanello
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(SFM)

file(GLOB header ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB source ${PROJECT_SOURCE_DIR}/src/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${stereoVision_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

find_package(IPOPT QUIET)
if(IPOPT_FOUND)
   message(STATUS "Ipopt found: setting up the project to enable Ipopt related features")
   file(GLOB header_ipopt ${PROJECT_SOURCE_DIR}/include/ipopt/*.h)
   file(GLOB source_ipopt ${PROJECT_SOURCE_DIR}/src/ipopt/*.cpp)
   list(APPEND header ${header_ipopt})
   list(APPEND source ${source_ipopt})

   include_directories(${IPOPT_INCLUDE_DIRS})
   add_definitions(${IPOPT_DEFINITIONS} -DUSING_IPOPT)
else()
   message(WARNING "Ipopt not found! Related features will be unavailable")   
endif()

if(USE_SIFT_GPU)
   include_directories(${SIFTGPU_INCLUDE_DIRS})
   add_definitions(-DUSING_GPU)
endif()

add_executable(${PROJECT_NAME} ${source} ${header})
target_link_libraries(${PROJECT_NAME} stereoVision iKin ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})

if(USE_SIFT_GPU)
   target_link_libraries(${PROJECT_NAME} ${SIFTGPU_LIBRARIES})
endif()

if(IPOPT_FOUND)
   target_link_libraries(${PROJECT_NAME} ${IPOPT_LIBRARIES})
   set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
