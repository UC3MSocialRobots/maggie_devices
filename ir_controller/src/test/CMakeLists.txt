add_executable(test_manual_ir_controller.exe
               test_manual_ir_controller.cpp ${catkin_LIBRARIES})
target_link_libraries(test_manual_ir_controller.exe
                      ir_controller_node ${catkin_LIBRARIES})

catkin_add_gtest(gtest_ir_controller_node.exe EXCLUDE_FROM_ALL
                 gtest_ir_controller_node.cpp)
target_link_libraries(gtest_ir_controller_node.exe
                      ir_controller_node gmock gtest ${catkin_LIBRARIES})
