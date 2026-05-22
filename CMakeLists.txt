cmake_minimum_required(VERSION 3.14)
project(EVThermalControl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
#  Fetch Google Test automatically
# ============================================================
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ============================================================
#  Core simulation library (all modules except main)
#  SocketServer added for Python dashboard integration
# ============================================================
add_library(ev_core
    src/SensorSimulator.cpp
    src/ThermalController.cpp
    src/CANBus.cpp
    src/DiagnosticsManager.cpp
    src/SocketServer.cpp
)
target_include_directories(ev_core PUBLIC src)

# Link Winsock on Windows (required for socket support)
if(WIN32)
    target_link_libraries(ev_core ws2_32)
endif()

# ============================================================
#  Main executable
# ============================================================
add_executable(EVThermalControl src/main.cpp)
target_link_libraries(EVThermalControl ev_core)

# ============================================================
#  Test executable
#  Note: SocketServer not included in tests — no network needed
# ============================================================
enable_testing()

add_executable(ev_tests
    tests/test_sensor.cpp
    tests/test_controller.cpp
    tests/test_diagnostics.cpp
)
target_link_libraries(ev_tests ev_core GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(ev_tests)