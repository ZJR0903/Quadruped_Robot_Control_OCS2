cmake_minimum_required(VERSION 3.10)
project(legged_hw)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-inconsistent-missing-override)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_common
        controller_manager
        urdf
        std_msgs
        message_generation
        # yesense_imu
        )

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        YesenseIMUSetting.msg
        # YesenseImuEulerAngle.msg
        # YesenseImuLocation.msg
        # YesenseImuQuaternion.msg
        # YesenseImuStatus.msg
        # YesenseImuUtcTime.msg
        # YesenseImuMasterGnssData.msg
        # YesenseImuSlaveGnssData.msg
        # YesenseImuGnssData.msg
        # YesenseImuGpsData.msg
        # YesenseImuSensorData.msg
        # YesenseImuAllData.msg
        # YesenseImuCmdResp.msg
      )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        legged_common
        controller_manager
        urdf
)


###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/hardware_interface.cpp
        src/control_loop.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
