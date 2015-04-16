catkin_add_gtest(gtest_motor_controller_node.exe EXCLUDE_FROM_ALL
                 gtest_motor_controller_node.cpp)
target_link_libraries(gtest_motor_controller_node.exe
                      motor_controller_node gmock gtest ${catkin_LIBRARIES})