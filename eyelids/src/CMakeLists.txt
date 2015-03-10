add_library(eyelids_node
            eyelids_node.cpp eyelids_node.h)
target_link_libraries(eyelids_node
                      serial_communication ${catkin_LIBRARIES})
add_dependencies(eyelids_node
                 ${eyelids_msgs_EXPORTED_TARGETS})

add_executable(eyelids_node_main.exe
               eyelids_node_main.cpp)
target_link_libraries(eyelids_node_main.exe
                      eyelids_node ${catkin_LIBRARIES})

install(TARGETS eyelids_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

ADD_SUBDIRECTORY(test)
