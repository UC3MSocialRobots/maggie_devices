add_library(base_motor_driver
            base_motor_driver.cpp base_motor_driver.h maggie_data.h)
target_link_libraries(base_motor_driver
                      m ${catkin_LIBRARIES})
                      
add_library(base_motor_node
            base_motor_node.cpp base_motor_node.h)
target_link_libraries(base_motor_node
                      base_motor_driver ${catkin_LIBRARIES})

add_executable(base_motor_node_main.exe 
			   base_motor_node_main.cpp)
target_link_libraries(base_motor_node_main.exe
                      base_motor_node ${catkin_LIBRARIES})

ADD_SUBDIRECTORY(test)

install(TARGETS base_motor_driver base_motor_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)