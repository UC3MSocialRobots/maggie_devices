add_library(labjack_node
            labjack_node.cpp labjack_node.h)
target_link_libraries(labjack_node
                      ljacklm_wrapper ${catkin_LIBRARIES})
add_dependencies(labjack_node
                 ${basic_states_skill_msgs_EXPORTED_TARGETS}
                 ${batteries_skill_msgs_EXPORTED_TARGETS}
                 ${touch_skill_msgs_EXPORTED_TARGETS})

add_executable(labjack_node_main.exe
               labjack_node_main.cpp)
target_link_libraries(labjack_node_main.exe
                      labjack_node ${catkin_LIBRARIES})

install(TARGETS labjack_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

ADD_SUBDIRECTORY(test)
