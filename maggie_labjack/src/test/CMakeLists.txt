add_executable(test_manual_labjack_primitives.exe
               test_manual_labjack_primitives.cpp)
target_link_libraries(test_manual_labjack_primitives.exe
                      ljacklm_wrapper ${catkin_LIBRARIES})

catkin_add_gtest(gtest_labjack_node.exe EXCLUDE_FROM_ALL
                 gtest_labjack_node.cpp)
target_link_libraries(gtest_labjack_node.exe
                      labjack_node gmock gtest ${catkin_LIBRARIES})
