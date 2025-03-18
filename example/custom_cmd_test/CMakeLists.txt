# CMakeLists.txt
#

cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 11)

project(custom_cmd_test)

set(EXECUTABLE_NAME ${PROJECT_NAME})

# 统一CMAKE_HOST_SYSTEM_PROCESSOR
string(REPLACE "AMD64" "x86_64" CMAKE_HOST_SYSTEM_PROCESSOR ${CMAKE_HOST_SYSTEM_PROCESSOR})


if(UNIX)
    # find_package(PkgConfig REQUIRED)
    find_package(PkgConfig REQUIRED)

    find_package(serial REQUIRED)

    # if (NOT serial_FOUND)
    #     message(FATAL_ERROR "serial Not Found!")
    # endif (NOT serial_FOUND)
elseif(WIN32)
    find_path(serial_INCLUDE_DIRS
        NAMES serial/serial.h
        HINTS /serial/include
        REQUIRED
    )

    find_path(serial_LIB_DIRS
        NAMES Release/serial.lib
        HINTS /serial/lib/
        REQUIRED
    )

    set(serial_LIBRARIES 
        debug ${serial_LIB_DIRS}/Debug/serial.lib
        optimized ${serial_LIB_DIRS}/Release/serial.lib
    )
else()
    message(ERROR "UNKNOWN COMPILER")
endif()

message ("serial_INCLUDE_DIRS: " ${serial_INCLUDE_DIRS})
message ("serial_LIBRARIES: " ${serial_LIBRARIES})


#
# --- Library dependencies ---

# find_path(XXX_INCLUDE_DIR
#     xxx.h
# )
# find_library(XXX_LIBRARY 
#     xxx
# )


## add sub projects
# add_subdirectory(sub_project_dir)

## include directories
include_directories(
    ${serial_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)


## link_directories
# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../lib)


## library
find_library(OHAND_LIBRARY_RELEASE
             NAMES ROHSerialAPIStatic
             HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/${CMAKE_HOST_SYSTEM_PROCESSOR}/${CMAKE_HOST_SYSTEM_NAME}/release)

find_library(OHAND_LIBRARY_DEBUG
             NAMES ROHSerialAPIStatic
             HINTS ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/${CMAKE_HOST_SYSTEM_PROCESSOR}/${CMAKE_HOST_SYSTEM_NAME}/debug)

set(ohand_LIBRARIES
    debug ${OHAND_LIBRARY_DEBUG}
    optimized ${OHAND_LIBRARY_RELEASE}
)

message ("ohand_LIBRARIES: " ${ohand_LIBRARIES})


## files
file(GLOB SRCS src/*.c src/*.cpp src/*.cc)  # a variable called SRCS with all files whose path match "*.c *.cpp..."

## binary files
add_executable(${PROJECT_NAME} ${SRCS})


## link binaries
target_link_libraries(${EXECUTABLE_NAME}
    # pthread
    ${serial_LIBRARIES}
    ${ohand_LIBRARIES}
)

set(INSTALL_TARGETS
    ${EXECUTABLE_NAME}
)

## install binaries
install(TARGETS ${INSTALL_TARGETS}
    RUNTIME DESTINATION bin
)
