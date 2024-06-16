#cmake_minimum_required(VERSION 3.13)

#set(NAME busterbot_feet)

#include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Pull in FreeRTOS
#include($ENV{FREERTOS_SDK_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

#SET(EIGEN_PATH "${CMAKE_CURRENT_LIST_DIR}/Eigen/include/eigen3/" CACHE STRING "Common Lib")

#SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}" CACHE STRING "Local Config")


#project(${NAME} C CXX ASM)
#set(CMAKE_C_STANDARD 11)
#set(CMAKE_CXX_STANDARD 17)

#pico_sdk_init()

#set(PICO_BOARD "pico" CACHE STRING "Pico board type")
#set(PICO_PLATFORM "rp2040" CACHE STRING "Pico platform type")

#link_directories(
#    $ENV{MICRO_ROS_PATH}/libmicroros
##    ${CMAKE_CURRENT_SOURCE_DIR}/Eigen/include/eigen3
#)

add_executable(${NAME}
    main.cpp
    GPIOInputMgr.cpp
    GPIOObserver.cpp
    MotorMgr.cpp
    MotorPID.cpp
    Agent.cpp
    BlinkAgent.cpp
    MotorsAgent.cpp
    Busterbot.cpp
    uRosBridge.cpp
    uRosEntities.cpp
    pico_usb_transport.c
    freertos_allocators.c
)


#add_library(eigen INTERFACE)

#target_include_directories(${NAME} PUBLIC
#    $ENV{MICRO_ROS_PATH}/libmicroros/include
#    $ENV{FREERTOS_SDK_PATH}/include
#    ${EIGEN_PATH}
#)



#target_link_libraries(${NAME}
#   pico_stdlib
 #   microros
 #   FreeRTOS-Kernel 
 #   FreeRTOS-Kernel-Heap4
  #  hardware_pwm
 #   pico_rand
#	eigen
##)


#target_compile_definitions(${NAME} PRIVATE
 #  configNUM_CORES=2
#)

# Generate UF2
pico_add_extra_outputs(${NAME})

pico_enable_stdio_uart(${NAME} 1)
pico_enable_stdio_usb(${NAME} 1)

#add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
#add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
#add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

target_compile_definitions(${NAME} PRIVATE 
    PICO_DEFAULT_UART_RX_PIN=16 #yellow
    PICO_DEFAULT_UART_TX_PIN=17 #red
)


