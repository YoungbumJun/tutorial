cmake_minimum_required(VERSION 2.6)

set(CMAKE_C_FLAGS "-Wall -g")
set(CMAKE_CXX_FLAGS "-Wall -g")

if(APPLE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wsign-compare")
  set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wsign-compare")
endif()

set(CMAKE_C_FLAGS_DEBUG "-O")
set(CMAKE_CXX_FLAGS_DEBUG "-O")

set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

include(FindPkgConfig)

pkg_search_module(EIGEN3 REQUIRED eigen3>=3)
find_path(HUBO_ACH_INCLUDE "hubo.h" PATHS ${CMAKE_INCLUDE_PATH})

if (${HUBO_ACH_INCLUDE} STREQUAL HUBO_ACH_INCLUDE-NOTFOUND)
  message("could not find hubo-ach includes")
  set(HAVE_HUBO_ACH 0)
else (${HUBO_ACH_INCLUDE} STREQUAL HUBO_ACH_INCLUDE-NOTFOUND)
  message("found hubo-ach includes at ${HUBO_ACH_INCLUDE}")
  add_definitions(-DHAVE_HUBO_ACH)
  set(HAVE_HUBO_ACH 1)
endif (${HUBO_ACH_INCLUDE} STREQUAL HUBO_ACH_INCLUDE-NOTFOUND)

# do not touch above

add_executable(tutorial src/tutorial.cpp)
target_link_libraries(tutorial ach)

