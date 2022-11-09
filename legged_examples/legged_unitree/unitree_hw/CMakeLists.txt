cmake_minimum_required(VERSION 3.10)
project(unitree_hw)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++11)
add_compile_options(-std=c++14)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-inconsistent-missing-override)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
        tf
        )

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        legged_hw
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo02_talker_listener
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

add_message_files(
  FILES
  YesenseIMUSetting.msg
  YesenseImuEulerAngle.msg
  YesenseImuLocation.msg
  YesenseImuQuaternion.msg
  YesenseImuStatus.msg
  YesenseImuUtcTime.msg
  YesenseImuMasterGnssData.msg
  YesenseImuSlaveGnssData.msg
  YesenseImuGnssData.msg
  YesenseImuGpsData.msg
  YesenseImuSensorData.msg
  YesenseImuAllData.msg
  YesenseImuCmdResp.msg
)


###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

set(EXTRA_LIBS libunitree_legged_sdk_amd64.so lcm)

## Declare cpp executables
add_executable(${PROJECT_NAME}
        src/${PROJECT_NAME}.cpp
        src/hardware_interface.cpp
        )

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${EXTRA_LIBS}
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

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
