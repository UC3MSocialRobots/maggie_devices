catkin_add_gtest(gtest_eyelids_node.exe EXCLUDE_FROM_ALL
                 gtest_eyelids_node.cpp)
target_link_libraries(gtest_eyelids_node.exe
                      eyelids_node gmock gtest ${catkin_LIBRARIES})
