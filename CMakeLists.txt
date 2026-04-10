cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME test_feedback)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# Collect User layer sources and include directories automatically
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/User/*.c
    ${CMAKE_SOURCE_DIR}/User/*.cpp
)

# Exclude duplicated legacy mirror sources to avoid multiple definition at link time.
list(FILTER USER_SOURCES EXCLUDE REGEX ".*/User/Algorithm/alg_filter\\.cpp$")
list(FILTER USER_SOURCES EXCLUDE REGEX ".*/User/Algorithm/alg_slope\\.cpp$")
list(FILTER USER_SOURCES EXCLUDE REGEX ".*/User/Driver/uart_printf\\.cpp$")
list(FILTER USER_SOURCES EXCLUDE REGEX ".*/User/Module/crt_wheelchassis\\.cpp$")

file(GLOB_RECURSE USER_HEADERS CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/User/*.h
    ${CMAKE_SOURCE_DIR}/User/*.hpp
)

set(USER_INCLUDE_DIRS)
foreach(USER_HEADER ${USER_HEADERS})
    get_filename_component(USER_INCLUDE_DIR ${USER_HEADER} DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS ${USER_INCLUDE_DIR})
endforeach()
list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)

# main.c includes C++ class headers from User layer, compile it as C++.
set_source_files_properties(
    ${CMAKE_SOURCE_DIR}/Core/Src/main.c
    PROPERTIES LANGUAGE CXX
)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${USER_INCLUDE_DIRS}
    Drivers/CMSIS/DSP/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Enable floating-point formatting for printf with newlib-nano.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
