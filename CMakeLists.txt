cmake_minimum_required(VERSION 3.22)
project(voxl-vision-hub)

set(CMAKE_C_FLAGS "-O2 -std=gnu99 -Wall -Wextra -Wuninitialized \
	-Wunused-variable -Wdouble-promotion -Wmissing-prototypes \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_C_FLAGS}")

# tell the linker not to worry about missing symbols in libraries
set(CMAKE_C_FLAGS   "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_CXX_FLAGS}")


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


if(PLATFORM STREQUAL QRB5165_2)
	message(STATUS "Building for platform QRB5165 2.X system images")
	add_definitions(-DPLATFORM_QRB5165_2)
elseif(PLATFORM STREQUAL QRB5165)
	message(STATUS "Building for platform QRB5165 1.X system images")
	add_definitions(-DPLATFORM_QRB5165)
else()
	message(FATAL_ERROR "Platform type not specified")
endif()

add_subdirectory (src)
add_subdirectory (utils)
add_subdirectory (voxl_trajectory)
