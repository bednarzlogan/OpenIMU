cmake_minimum_required(VERSION 3.20)
project(ukf_realtime LANGUAGES CXX)

option(UKF_BUILD_SIM       "Build timed playback simulator lib" ON)
option(UKF_BUILD_SIM_APP   "Build a runnable playback app"      ON)
option(UKF_BUILD_TESTS     "Build GoogleTest-based tests"       OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

# ========= Core library =========
add_library(ukf_core STATIC
  ukf_core/src/UKF.cpp
)
target_include_directories(ukf_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/ukf_core/include
)
target_link_libraries(ukf_core PUBLIC
  Eigen3::Eigen
  Boost::boost
  Threads::Threads
)
add_library(ukf::core ALIAS ukf_core)

# ========= Simulator library (optional) =========
if(UKF_BUILD_SIM)
  add_library(ukf_sim STATIC
    sim/src/timed_playback_sim.cpp
  )
  target_include_directories(ukf_sim PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/sim/include
  )
  target_link_libraries(ukf_sim PUBLIC
    ukf::core
    Eigen3::Eigen
    Boost::boost
    Threads::Threads
  )
  add_library(ukf::sim ALIAS ukf_sim)
endif()

# ========= Runnable app (optional) =========
if(UKF_BUILD_SIM AND UKF_BUILD_SIM_APP)
  add_executable(ukf_playback main.cpp)
  target_link_libraries(ukf_playback PRIVATE ukf::sim Threads::Threads)
  add_custom_command(TARGET ukf_playback POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/config
            $<TARGET_FILE_DIR:ukf_playback>/config
  )
endif()

# ========= Tests (optional) =========
if(UKF_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_executable(ukf_tests
    tests/test_retrodict.cpp
    tests/test_stale_gnss.cpp
    tests/test_sim_pump.cpp
    # Option A: no explicit test_main; use gtest_main:
    # (remove tests/test_main.cpp if it existed)
  )
  target_link_libraries(ukf_tests PRIVATE
    ukf::core
    GTest::gtest_main
  )
  if(UKF_BUILD_SIM)
    target_link_libraries(ukf_tests PRIVATE ukf::sim)
  endif()

  add_custom_command(TARGET ukf_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/config
            $<TARGET_FILE_DIR:ukf_tests>/config
  )

  include(GoogleTest)
  gtest_discover_tests(ukf_tests)
endif()
