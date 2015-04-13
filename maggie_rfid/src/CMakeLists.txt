add_library(rfid_node
            rfid_node.cpp rfid_node.h)
target_link_libraries(rfid_node
                      rfid_driver_wrapper ${catkin_LIBRARIES})
add_dependencies(rfid_node
                 ${maggie_rfid_msgs_EXPORTED_TARGETS})

add_executable(rfid_node_main.exe
               rfid_node_main.cpp)
target_link_libraries(rfid_node_main.exe
                      rfid_node ${catkin_LIBRARIES})

install(TARGETS rfid_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#ADD_SUBDIRECTORY(test)
