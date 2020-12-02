project(common)

include_directories(${catkin_INCLUDE_DIRS})

add_library(pid_controller
            pid_controller.h 
            pid_controller.cpp )


add_subdirectory(test)