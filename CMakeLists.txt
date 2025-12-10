cmake_minimum_required(VERSION 3.20)

if(NOT CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
endif()

project(mesh-simplifier VERSION 0.1.0 LANGUAGES CXX)

# Set a sane default C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(mesh-simplifier)

if(NOT TARGET main)
	add_executable(main ALIAS mesh-simplifier)
endif()

target_sources(mesh-simplifier 
	PRIVATE 
		src/main.cpp
		src/nanite.cpp
)

# Include directories for the main application
target_include_directories(mesh-simplifier 
	PRIVATE 
		${CMAKE_CURRENT_SOURCE_DIR}/src
		${CMAKE_CURRENT_SOURCE_DIR}/lib
)

add_compile_definitions(_CRT_SECURE_NO_WARNINGS _SCL_SECURE_NO_WARNINGS)
if(MSVC)
	target_compile_options(mesh-simplifier PRIVATE /wd4172)
else()
	target_compile_options(mesh-simplifier PRIVATE -Wno-return-stack-address -Wno-error=return-stack-address -Wno-deprecated-declarations)
endif()

find_path(VCGLIB_INCLUDE_DIRS "img/img.h")
target_include_directories(mesh-simplifier PRIVATE ${VCGLIB_INCLUDE_DIRS})

message(STATUS "Project: ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")