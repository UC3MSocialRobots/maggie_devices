add_executable(test_manual_base_motors.exe
               test_manual_base_motors.cpp)
target_link_libraries(test_manual_base_motors.exe
                      base_motor_driver ${catkin_LIBRARIES})
