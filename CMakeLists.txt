cmake_minimum_required(VERSION 3.10)

project(commuties)

FIND_PACKAGE( Boost COMPONENTS program_options graph REQUIRED )
INCLUDE_DIRECTORIES( "${Boost_INCLUDE_DIR}" )

if(NOT Boost_FOUND)
  MESSAGE(FATAL_ERROR "Could not find Boost.")
endif(NOT Boost_FOUND)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

file(GLOB SOURCES "src/*.cpp")
include_directories ("${PROJECT_SOURCE_DIR}")
set(SOURCEs src/main.cpp)

# static linking
#if(${STATIC} STREQUAL ON)
#  set(Boost_USE_STATIC_LIBS   ON)
#  SET(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
#  SET(BUILD_SHARED_LIBRARIES OFF)
#  SET(CMAKE_EXE_LINKER_FLAGS "-static")
#endif(${STATIC} STREQUAL ON)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_COMMON_FLAGS "${CMAKE_COMMON_FLAGS} -pthread -Wall -std=c++11 -DIL_STD")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_COMMON_FLAGS} -O3 -march=native -DNDEBUG -pipe -m64 -msahf -fno-stack-protector")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_COMMON_FLAGS} -g -march=native -fno-stack-protector")

ADD_EXECUTABLE(2cs src/main.cpp)
TARGET_LINK_LIBRARIES(2cs ${Boost_LIBRARIES})
