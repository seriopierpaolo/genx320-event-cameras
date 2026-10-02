cmake_minimum_required(VERSION 3.16)
project(genx320_simulator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall")

# Source files
set(SOURCES
    src/main.cpp
    src/simulator.cpp
    src/command_handler.cpp
    src/event_streamer.cpp
)

add_executable(genx320_simulator ${SOURCES})

target_include_directories(genx320_simulator PRIVATE include)

# pthreads for std::thread
find_package(Threads REQUIRED)
target_link_libraries(genx320_simulator PRIVATE Threads::Threads)

# -----------------------------------------------------------------------
# genx320_client – connects to the simulator, buffers events, runs Hough
# -----------------------------------------------------------------------
find_package(OpenCV REQUIRED)

set(CLIENT_SOURCES
    src/client_main.cpp
    src/line_detector.cpp
)

add_executable(genx320_client ${CLIENT_SOURCES})

target_include_directories(genx320_client PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(genx320_client PRIVATE Threads::Threads ${OpenCV_LIBS})

# -----------------------------------------------------------------------
# Copy sample events file next to the binary
configure_file(
    ${CMAKE_SOURCE_DIR}/data/vertical_line.txt
    ${CMAKE_BINARY_DIR}/vertical_line.txt
    COPYONLY
)

# -----------------------------------------------------------------------
# Tests – Google Test
# -----------------------------------------------------------------------
enable_testing()
find_package(GTest REQUIRED)

# Copy ground truth data next to the test binary
configure_file(
    ${CMAKE_SOURCE_DIR}/data/ground_truth.txt
    ${CMAKE_BINARY_DIR}/ground_truth.txt
    COPYONLY
)

add_executable(genx320_tests
    test/test_line_detector.cpp
    src/line_detector.cpp
)

target_include_directories(genx320_tests PRIVATE include)
target_link_libraries(genx320_tests PRIVATE
    GTest::gtest_main
    Threads::Threads
)

include(GoogleTest)
gtest_discover_tests(genx320_tests)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Binary will be: ${CMAKE_BINARY_DIR}/genx320_simulator")
