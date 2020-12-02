include_directories(${catkin_INCLUDE_DIRS})

catkin_add_gtest(mean_filter-test 
                mean_filter_test.cpp )
if(TARGET mean_filter-test)
    target_link_libraries(mean_filter-test
                          mean_filter
                          ${catkin_LIBRARIES} )
endif()

catkin_add_gtest(digital_filter-test 
                 digital_filter_test.cpp )
if (TARGET digital_filter-test)
    target_link_libraries(digital_filter-test
                          digital_filter
                          ${catkin_LIBRARIES} )
endif()

catkin_add_gtest(digital_filter_coefficients-test 
                 digital_filter_coefficients_test.cpp)
if (TARGET digital_filter_coefficients-test)
    target_link_libraries(digital_filter_coefficients-test
                          digital_filter_coefficients
                          ${catkin_LIBRARIES} )
endif()