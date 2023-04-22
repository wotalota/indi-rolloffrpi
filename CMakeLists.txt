cmake_minimum_required(VERSION 3.5)
project(indi_rolloffrpi C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")
include(GNUInstallDirs)

set(INDI_ROLLOFFINO_VERSION_MAJOR 1)
set(INDI_ROLLOFFINO_VERSION_MINOR 0)
set(GPIO_LIBRARY "pigpiod_if2.so")

find_package(INDI REQUIRED)
find_package(Nova)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake              ${CMAKE_CURRENT_BINARY_DIR}/config.h)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_rolloffinorpi.xml.cmake   ${CMAKE_CURRENT_BINARY_DIR}/indi_rolloffrpi.xml)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${INDI_INCLUDE_DIR})

include(CMakeCommon)

set(indirolloffrpi_SRCS
   ${CMAKE_CURRENT_SOURCE_DIR}/rolloffrpi.cpp
)

add_executable(indi_rolloffrpi ${indirolloffrpi_SRCS})

target_link_libraries(indi_rolloffrpi
target_link_libraries(indi_rolloffrpi ${GPIO_LIBRARY})

install(TARGETS indi_rolloffrpi RUNTIME DESTINATION bin )
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_rolloffrpi.xml DESTINATION ${INDI_DATA_DIR})






