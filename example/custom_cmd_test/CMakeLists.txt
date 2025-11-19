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
    
    find_path(pcanbasic_INCLUDE_DIRS
    	NAMES PCANBasic.h
    	HINTS /PCAN-Basic_Linux-4.10.0.4/libpcanbasic/pcanbasic/include
    	REQUIRED
    )
    find_library(pcanbasic_LIBRARIES
    	NAMES libpcanbasic.so
    	HINTS /PCAN-Basic_Linux-4.10.0.4/libpcanbasic/pcanbasic/lib
    	REQUIRED
    )
    
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

    find_path(pcanbasic_INCLUDE_DIRS
        NAMES PCANBasic.h
        HINTS /PCAN-Basic/Include
        REQUIRED
    )

    find_path(pcanbasic_LIB_DIRS
        NAMES PCANBasic.lib
        HINTS /PCAN-Basic/x64/VC_LIB
        REQUIRED
    )

    set(pcanbasic_LIBRARIES 
        ${pcanbasic_LIB_DIRS}/PCANBasic.lib
    )

else()
    message(ERROR "UNKNOWN COMPILER")
endif()

message ("serial_INCLUDE_DIRS: " ${serial_INCLUDE_DIRS})
message ("serial_LIB_DIRS: " ${serial_LIB_DIRS})
message ("serial_LIBRARIES: " ${serial_LIBRARIES})

message ("pcanbasic_INCLUDE_DIRS: " ${pcanbasic_INCLUDE_DIRS})
message ("pcanbasic_LIB_DIRS: " ${pcanbasic_LIB_DIRS})
message ("pcanbasic_LIBRARIES: " ${pcanbasic_LIBRARIES})


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
    ${pcanbasic_INCLUDE_DIRS}
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

# get git version
find_package(Git QUIET)
if(Git_FOUND)
    # get current commit hash
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_COMMIT_HASH_SHORT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    # get current branch
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_BRANCH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    # get current describe info
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags --long --always
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_DESCRIBE
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    # combine version
    if(GIT_DESCRIBE)
        string(REGEX REPLACE "^([0-9]+\\.[0-9]+)-([0-9]+)-g.*" "\\1" GIT_TAG ${GIT_DESCRIBE})
        string(REGEX REPLACE "^([0-9]+\\.[0-9]+)-([0-9]+)-g.*" "\\2" GIT_COMMITS_SINCE_TAG ${GIT_DESCRIBE})

        set(VERSION_STRING "${GIT_TAG}.${GIT_COMMITS_SINCE_TAG}-${GIT_COMMIT_HASH_SHORT}")
    else()
      set(VERSION_STRING "${GIT_BRANCH}-${GIT_COMMIT_HASH_SHORT}")
    endif()
else()
    set(VERSION_STRING "unknow")
endif()

## binary files
add_executable(${PROJECT_NAME} ${SRCS})

target_compile_definitions(${PROJECT_NAME} PRIVATE VERSION_STRING="${VERSION_STRING}")

## link binaries
target_link_libraries(${EXECUTABLE_NAME}
    # pthread
    ${serial_LIBRARIES}
    ${pcanbasic_LIBRARIES}
    ${ohand_LIBRARIES}
)

set(INSTALL_TARGETS
    ${EXECUTABLE_NAME}
)

## install binaries
install(TARGETS ${INSTALL_TARGETS}
    RUNTIME DESTINATION bin
)
