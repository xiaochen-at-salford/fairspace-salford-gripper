include_directories(${catkin_INCLUDE_DIRS})

catkin_add_gtest(pid_controller-test 
                 pid_controller_test.cpp)
if (TARGET pid_controller-test)
    target_link_libraries(pid_controller-test
                          pid_controller
                          ${catkin_LIBRARIES} )
endif()