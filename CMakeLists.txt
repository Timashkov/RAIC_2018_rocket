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
        csimplesocket/ActiveSocket.h
        csimplesocket/Host.h
        csimplesocket/HTTPActiveSocket.cpp
        csimplesocket/HTTPActiveSocket.h
        csimplesocket/PassiveSocket.cpp
        csimplesocket/PassiveSocket.h
        csimplesocket/SimpleSocket.cpp
        csimplesocket/SimpleSocket.h
        csimplesocket/StatTimer.h
        model/Action.h
        model/Arena.h
        model/Ball.h
        model/Game.h
        model/NitroPack.h
        model/Player.h
        model/Robot.h
        model/Rules.h
        rapidjson/error/en.h
        rapidjson/error/error.h
        rapidjson/internal/biginteger.h
        rapidjson/internal/diyfp.h
        rapidjson/internal/dtoa.h
        rapidjson/internal/ieee754.h
        rapidjson/internal/itoa.h
        rapidjson/internal/meta.h
        rapidjson/internal/pow10.h
        rapidjson/internal/regex.h
        rapidjson/internal/stack.h
        rapidjson/internal/strfunc.h
        rapidjson/internal/strtod.h
        rapidjson/internal/swap.h
        rapidjson/allocators.h
        rapidjson/cursorstreamwrapper.h
        rapidjson/document.h
        rapidjson/encodedstream.h
        rapidjson/encodings.h
        rapidjson/filereadstream.h
        rapidjson/filewritestream.h
        rapidjson/fwd.h
        rapidjson/istreamwrapper.h
        rapidjson/memorybuffer.h
        rapidjson/memorystream.h
        rapidjson/ostreamwrapper.h
        rapidjson/pointer.h
        rapidjson/prettywriter.h
        rapidjson/rapidjson.h
        rapidjson/reader.h
        rapidjson/schema.h
        rapidjson/stream.h
        rapidjson/stringbuffer.h
        rapidjson/writer.h
        MyStrategy.cpp
        MyStrategy.h
        RemoteProcessClient.cpp
        RemoteProcessClient.h
        Runner.cpp
        Runner.h
        Strategy.cpp
        Strategy.h strategy_impl/VecUtils.h strategy_impl/Entity.h)

add_definitions (-D_LINUX)
add_compile_options( -std=c++17 -static -fno-optimize-sibling-calls -fno-strict-aliasing -lm -s -x c++ -O2 -Wall -Wtype-limits -Wno-unknown-pragmas)
#add_library(m)

#set(GCC_COVERAGE_COMPILE_FLAGS " -static -fno-optimize-sibling-calls -fno-strict-aliasing -lm -s -x c++ -O2 -Wall -Wtype-limits -Wno-unknown-pragmas")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
