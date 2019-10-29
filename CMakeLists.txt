# cmake_minimum_required(VERSION <specify CMake version here>)
cmake_minimum_required(VERSION 3.13)
project(socket_client)

set(CMAKE_CXX_STANDARD 14)
# link_libraries(ws2_32)
# target_link_libraries(WS2_32.lib)
add_executable(socket_client main.cpp trafficFlowSocket.cpp trafficFlowSocket.h PLANNER/Function.cpp PLANNER/H.cpp)
target_link_libraries(socket_client -ldl -lpthread)
