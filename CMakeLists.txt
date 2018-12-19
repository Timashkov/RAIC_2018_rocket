cmake_minimum_required(VERSION 3.12)
project(strategy)

set(CMAKE_CXX_STANDARD 17)

include_directories(.)
include_directories(csimplesocket)
include_directories(model)
include_directories(rapidjson)
include_directories(rapidjson/error)
include_directories(rapidjson/internal)
include_directories(strategy_impl)

add_executable(strategy
        csimplesocket/ActiveSocket.cpp
        csimplesocket/HTTPActiveSocket.cpp
        csimplesocket/PassiveSocket.cpp
        csimplesocket/SimpleSocket.cpp
        MyStrategy.cpp
        RemoteProcessClient.cpp
        Runner.cpp
        Strategy.cpp)

add_definitions(-D_LINUX -DLOCAL_RUN)
add_compile_options(-std=c++17 -static -fno-optimize-sibling-calls -fno-strict-aliasing -lm -s -x c++ -O2 -Wall -Wtype-limits -Wno-unknown-pragmas)
#add_library(m)

#set(GCC_COVERAGE_COMPILE_FLAGS " -static -fno-optimize-sibling-calls -fno-strict-aliasing -lm -s -x c++ -O2 -Wall -Wtype-limits -Wno-unknown-pragmas")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
