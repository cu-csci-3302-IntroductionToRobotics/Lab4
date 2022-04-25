cmake_minimum_required(VERSION 3.0)

# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

file(GLOB SOURCES apps/*.cpp code/*.cpp sim/*.h)

message("env is : $ENV{WEBOTS_HOME}")

set( WEBOTS_HOME $ENV{WEBOTS_HOME})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/modules)
set(IS_ENV_WEBOTS_HOME $ENV{WEBOTS_HOME})
find_package(Webots REQUIRED)

# message("libs-> ${WEBOTS_LIBRARIES}")
# message("libs-> ${WEBOTS_LIBRARY_CPP}")
# message("libs-> ${WEBOTS_LIBRARY_C}")

#find opencv
find_package(OpenCV REQUIRED)

include_directories(
    ${WEBOTS_INCLUDE_CPP_DIR}
    ${OpenCV_INCLUDE_DIRS}
)


add_executable(run_lab ${SOURCES})
target_link_libraries(run_lab ${WEBOTS_LIBRARIES} ${OpenCV_LIBRARIES})
