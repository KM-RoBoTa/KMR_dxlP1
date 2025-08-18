cmake_minimum_required(VERSION 3.20)
project(KMR_dxlP1)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Source files
add_library(${PROJECT_NAME} 
            source/motor_handler.cpp
            source/handler.cpp
            #source/reader.cpp
            source/writer.cpp
            source/hal.cpp
            source/utils.cpp
        )

# Directories containing header files
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Locations of the used libraries
target_link_directories(${PROJECT_NAME} PUBLIC /usr/local/lib)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    message(STATUS "Processor used: " ${CMAKE_SYSTEM_PROCESSOR})
    target_link_libraries(${PROJECT_NAME} dxl_x64_cpp)
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l")
    message(STATUS "Processor used: " ${CMAKE_SYSTEM_PROCESSOR})
    target_link_libraries(${PROJECT_NAME} dxl_sbc_cpp)
    elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    message(STATUS "Processor used: " ${CMAKE_SYSTEM_PROCESSOR})
    target_link_libraries(${PROJECT_NAME} dxl_sbc_cpp)
else()
    message(FATAL_ERROR "ERROR: Unknown processor architecture! CMake will exit")
endif()

# Generate Docs
option(BUILD_DOCS "Generate Docs" ON)
if(BUILD_DOCS)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/docs)  # path to docs' cmakelists
endif()


# Generate example executables
add_executable(ex1_position examples/ex1_position.cpp)
target_link_libraries(ex1_position ${PROJECT_NAME})