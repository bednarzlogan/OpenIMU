# specify version requirements and project name 
cmake_minimum_required(VERSION 3.10)
project(IMU_model)

# set a C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# generates a compile commands json that can integrate with clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set sim mode on
add_compile_definitions(SIM_MODE)

# add exec target 
add_executable(
  IMU_model
  src/main.cpp 
  src/IMU.cpp
  src/UKF.cpp
  src/measurement_handler.cpp
)

# add include target_include_directories
target_include_directories(IMU_model PRIVATE include)

# we'll require Eigen for this 
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Link the project to Eigen 
target_include_directories(IMU_model PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_compile_definitions(IMU_model PRIVATE ${EIGEN3_DEFINITIONS})

# link POSIX for std thread 
target_link_libraries(IMU_model PRIVATE pthread)

