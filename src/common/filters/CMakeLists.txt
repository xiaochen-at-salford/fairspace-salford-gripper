# SUBM=PROJECT_NAME
project(filters)


include_directories(${catkin_INCLUDE_DIRS})

add_library(mean_filter
            mean_filter.h
            mean_filter.cpp )

add_library(digital_filter
            digital_filter.cpp
            digital_filter.h )

add_library(digital_filter_coefficients
            digital_filter_coefficients.cpp
            digital_filter_coefficients.h )

add_subdirectory(test)