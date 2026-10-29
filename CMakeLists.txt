cmake_minimum_required(VERSION 3.16)
project(automotive_ecu_freertos LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

# Simulation kernel + OSAL host backend (FreeRTOS backend lives in port/)
add_library(simkernel src/sim/kernel.cpp src/osal/osal_sim.cpp)
target_include_directories(simkernel PUBLIC include)

# CAN FD frame model and bus
add_library(canfd src/canfd/frame.cpp src/canfd/bus.cpp)
target_link_libraries(canfd PUBLIC simkernel)

# ECU firmware (portable: only uses osal.h and the CAN controller model)
add_library(ecu_fw
  src/ecu/dtc.cpp
  src/ecu/uds.cpp
  src/ecu/sensors.cpp
  src/ecu/messages.cpp
  src/ecu/metrics.cpp
  src/ecu/ecu_node.cpp
  src/ecu/network.cpp)
target_link_libraries(ecu_fw PUBLIC canfd)

add_executable(network_sim tools/network_sim.cpp)
target_link_libraries(network_sim PRIVATE ecu_fw)

enable_testing()
foreach(t test_kernel test_canfd test_bus test_dtc_uds test_sensors test_ecu test_network)
  add_executable(${t} tests/${t}.cpp)
  target_link_libraries(${t} PRIVATE ecu_fw)
  target_include_directories(${t} PRIVATE tests)
  add_test(NAME ${t} COMMAND ${t})
endforeach()
