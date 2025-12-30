cmake_minimum_required(VERSION 3.9)
set(SOEM_DIR "${CMAKE_CURRENT_LIST_DIR}/soem")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${SOEM_DIR}/cmake/Modules")

project(soem
        DESCRIPTION "Simple Open EtherCAT Master"
        VERSION 1.4.0
        LANGUAGES C
)

if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)


if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_LIST_DIR}/install)
endif()

set(SOEM_INCLUDE_INSTALL_DIR include)
set(SOEM_LIB_INSTALL_DIR lib)


if(WIN32)
    set(OS "win32")
    include_directories(${SOEM_DIR}/oshw/win32/wpcap/Include)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        link_directories(${SOEM_DIR}/oshw/win32/wpcap/Lib/x64)
    elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
        link_directories(${SOEM_DIR}/oshw/win32/wpcap/Lib)
    endif()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /D _CRT_SECURE_NO_WARNINGS /WX")
    set(OS_LIBS wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX AND NOT APPLE)
    set(OS "linux")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
    set(OS_LIBS pthread rt)
elseif(APPLE)
    set(OS "macosx")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
    set(OS_LIBS pthread pcap)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
    set(OS "rtk")
    message(STATUS "ARCH is ${ARCH}")
    message(STATUS "BSP is ${BSP}")
    include_directories(${SOEM_DIR}/oshw/${OS}/${ARCH})
    file(GLOB OSHW_EXTRA_SOURCES ${SOEM_DIR}/oshw/${OS}/${ARCH}/*.c)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror -Wno-unused-but-set-variable -Wno-unused-function -Wno-format")
    set(OS_LIBS "-Wl,--start-group -l${BSP} -l${ARCH} -lkern -ldev -lsio -lblock -lfs -lusb -llwip -leth -li2c -lrtc -lcan -lnand -lspi -lnor -lpwm -ladc -ltrace -lc -lm -Wl,--end-group")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rtems")
    message(STATUS "Building for RTEMS")
    set(OS "rtems")
    set(SOEM_LIB_INSTALL_DIR ${LIB_DIR})
    set(BUILD_TESTS FALSE)
endif()

message(STATUS "Detected OS is ${OS}")

file(GLOB SOEM_SOURCES ${SOEM_DIR}/soem/*.c)
file(GLOB OSAL_SOURCES ${SOEM_DIR}/osal/${OS}/*.c)
file(GLOB OSHW_SOURCES ${SOEM_DIR}/oshw/${OS}/*.c)

file(GLOB SOEM_HEADERS ${SOEM_DIR}/soem/*.h)
file(GLOB OSAL_HEADERS ${SOEM_DIR}/osal/osal.h ${SOEM_DIR}/osal/${OS}/*.h)
file(GLOB OSHW_HEADERS ${SOEM_DIR}/oshw/${OS}/*.h)

add_library(soem STATIC
        ${SOEM_SOURCES}
        ${OSAL_SOURCES}
        ${OSHW_SOURCES}
        ${OSHW_EXTRA_SOURCES}
)

target_link_libraries(soem ${OS_LIBS})

target_include_directories(soem PUBLIC
        $<BUILD_INTERFACE:${SOEM_DIR}/soem>
        $<BUILD_INTERFACE:${SOEM_DIR}/osal>
        $<BUILD_INTERFACE:${SOEM_DIR}/osal/${OS}>
        $<BUILD_INTERFACE:${SOEM_DIR}/oshw/${OS}>
        $<INSTALL_INTERFACE:${SOEM_INCLUDE_INSTALL_DIR}>
)

ament_export_targets(soem_EXPORT HAS_LIBRARY_TARGET)

install(FILES
        ${SOEM_HEADERS}
        ${OSAL_HEADERS}
        ${OSHW_HEADERS}
        DESTINATION ${SOEM_INCLUDE_INSTALL_DIR}
)

install(TARGETS soem
        EXPORT soem_EXPORT
        LIBRARY DESTINATION ${SOEM_LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${SOEM_LIB_INSTALL_DIR}
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION ${SOEM_INCLUDE_INSTALL_DIR}
)

ament_package()