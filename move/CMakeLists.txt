cmake_minimum_required(VERSION 2.8.3)
project(move)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    message_generation
)

add_message_files(DIRECTORY msg
    FILES
    gyro.msg
    sdktask.msg
    sdkstatus.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
include(CheckIncludeFiles)
check_include_files(linux/joystick.h HAVE_LINUX_JOYSTICK_H)

if(HAVE_LINUX_JOYSTICK_H)
    include_directories(msg/cpp ${catkin_INCLUDE_DIRS})
    add_executable(joy_node src/joy_node.cpp)
    target_link_libraries(joy_node ${catkin_LIBRARIES})
else(HAVE_LINUX_JOYSTICK_H)
    message("Warning: no <linux/joystick.h>; won't build joy node")
endif(HAVE_LINUX_JOYSTICK_H)

add_executable(sdk_slam_serial src/sdk_slam_serial.cpp)
target_link_libraries(sdk_slam_serial ${catkin_LIBRARIES})

add_executable(js_turtle_node src/js_turtle.cpp)
target_link_libraries(js_turtle_node ${catkin_LIBRARIES})

if(HAVE_LINUX_JOYSTICK_H)
    install(TARGETS joy_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif(HAVE_LINUX_JOYSTICK_H)

install(DIRECTORY migration_rules
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

