add_library(motor_controller_node
            motor_controller_node.cpp motor_controller_node.h)
target_link_libraries(motor_controller_node
                      mcdc3006s rs232 ${catkin_LIBRARIES})
add_dependencies(motor_controller_node
                 ${maggie_motor_controller_msgs_EXPORTED_TARGETS})

add_executable(motor_controller_node_main.exe
               motor_controller_node_main.cpp)
target_link_libraries(motor_controller_node_main.exe
                      motor_controller_node ${catkin_LIBRARIES})

install(TARGETS motor_controller_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#ADD_SUBDIRECTORY(test)
