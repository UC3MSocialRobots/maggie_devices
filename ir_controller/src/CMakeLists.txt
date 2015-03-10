add_library(ir_controller_node
            ir_controller_node.cpp)
target_link_libraries(ir_controller_node
                      irtrans_wrapper ${catkin_LIBRARIES})
add_dependencies(ir_controller_node
                 ${ir_controller_msgs_EXPORTED_TARGETS})

add_executable(ir_controller_node_main.exe
               ir_controller_node_main.cpp ${catkin_LIBRARIES})
target_link_libraries(ir_controller_node_main.exe
                      ir_controller_node)

install(TARGETS ir_controller_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

ADD_SUBDIRECTORY(test)
