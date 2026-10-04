cmake_minimum_required(VERSION 3.14)
project(AGVCollisionAvoidance VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# Include directories
include_directories(include)

# Source files for core library
set(CORE_SOURCES
    src/simulation/Vector2.cpp
    src/simulation/Pose.cpp
    src/simulation/AGV.cpp
    src/simulation/Obstacle.cpp
    src/simulation/World.cpp
    src/sensor/SensorData.cpp
    src/sensor/VirtualSensor.cpp
    src/sensor/LinuxSensorInterface.cpp
    src/collision/Distance.cpp
    src/collision/StoppingDistance.cpp
    src/collision/TTC.cpp
    src/collision/RiskEngine.cpp
    src/controller/SafetyStateMachine.cpp
    src/controller/MotionController.cpp
    src/logging/Logger.cpp
)

# Core static library
add_library(agv_safety_core STATIC ${CORE_SOURCES})

# Main simulation application executable
add_executable(agv_sim src/main.cpp)
target_link_libraries(agv_sim PRIVATE agv_safety_core)

# Test Runner executable
set(TEST_SOURCES
    tests/TestRunner.cpp
)
add_executable(agv_tests ${TEST_SOURCES})
target_link_libraries(agv_tests PRIVATE agv_safety_core)

# Enable CTest
enable_testing()
add_test(NAME AGVSafetyTestSuite COMMAND agv_tests)
