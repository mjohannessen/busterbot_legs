cmake_minimum_required(VERSION 3.13)

# Change your executable name to something creative!
set(NAME busterbot_feet) # <-- Name your project/executable here!

#include(pico_sdk_import.cmake)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

set(PICO_BOARD "pico" CACHE STRING "Pico board type")
set(PICO_PLATFORM "rp2040" CACHE STRING "Pico platform type")


include($ENV{FREERTOS_SDK_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
#SET(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/../../lib/FreeRTOS-Kernel" CACHE STRING "Common Lib")
SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel" CACHE STRING "Local Config")
include_directories("${FREERTOS_CONFIG_FILE_DIRECTORY}") 
#(FreeRTOS_Kernel_import.cmake)


SET(EIGEN_PATH "${CMAKE_CURRENT_LIST_DIR}/Eigen/include/eigen3/" CACHE STRING "Common Lib")

#SET(MICRO_ROS_PATH "${CMAKE_CURRENT_LIST_DIR}/../../lib/micro_ros_raspberrypi_pico_sdk/" CACHE STRING "Common Lib")
#SET(MICRO_ROS_PATH $ENV{MICRO_ROS_PATH}/libmicroros" CACHE STRING "Common Lib")
#(micro_ros.cmake)

#SET(EIGEN_PATH "${CMAKE_CURRENT_LIST_DIR}/../../lib/eigen/" CACHE STRING "Common Lib")
#include(eigen_import.cmake)

link_directories(
    $ENV{MICRO_ROS_PATH}/libmicroros
    ${CMAKE_CURRENT_SOURCE_DIR}/Eigen/include/eigen3
)


add_subdirectory(src)

add_library(eigen INTERFACE)

target_include_directories(${NAME} PUBLIC
    $ENV{MICRO_ROS_PATH}/libmicroros/include
    $ENV{FREERTOS_SDK_PATH}/include
    ${EIGEN_PATH}
)



target_link_libraries(${NAME}
    pico_stdlib
    microros
    FreeRTOS-Kernel 
    FreeRTOS-Kernel-Heap4
    hardware_pwm
    pico_rand
	eigen
)

#Set up files for the release packages
#install(CODE "execute_process(COMMAND $ENV{HOME}/bin/picoDeploy.sh ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.elf)")

# Set up files for the release packages
#install(FILES
#    ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.uf2
 #   DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
#)

#set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
#set(CPACK_GENERATOR "ZIP" "TGZ")
#include(CPack)
