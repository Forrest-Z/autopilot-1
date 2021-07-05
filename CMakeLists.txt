cmake_minimum_required(VERSION 3.5.1)
project(S99USV_ARM VERSION 0.1.0)

#设置执行文件输出目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#设置库输出路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_SYSTEM_NAME Linux)

#option
option(use_linux "to switch build linux target" OFF)
option(use_arm "to switch build arm target" OFF)


if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-w)
    add_compile_options(-std=c++11)
    add_compile_options(-Wall -O2 -c) #g
    add_compile_options(-D _GLIBCXX_USE_NANOSLEEP)
    message(STATUS "optional:-std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)

if(use_linux)
	message(STATUS "linux compiler")
else()
    SET(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchain.cmake)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)                
    set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)         
    set(CMAKE_INSTALL_RPATH "")                      
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)     
    message(STATUS "arm-linux compiler")
endif()


include_directories(
    .
    ${CMAKE_SOURCE_DIR}/3rdLibs/
    ${CMAKE_SOURCE_DIR}/3rdLibs/libiconv/include/
    ${CMAKE_SOURCE_DIR}/3rdLibs/nanomsg/include/
    ${CMAKE_SOURCE_DIR}/3rdLibs/protobuf/include/
    ${CMAKE_SOURCE_DIR}/3rdLibs/modbus/include/
    ${CMAKE_SOURCE_DIR}/3rdLibs/zmq/include/

    ${CMAKE_SOURCE_DIR}/3rdLibs/eigen/
    ${CMAKE_SOURCE_DIR}/3rdLibs/eigen/include/
    ${CMAKE_SOURCE_DIR}/3rdLibs/eigen/include/eigen3/
    ${CMAKE_SOURCE_DIR}/3rdLibs/eigen/include/eigen3/Eigen/

    ${CMAKE_SOURCE_DIR}/3rdLibs/xml/
    ${CMAKE_SOURCE_DIR}/3rdLibs/xml/include/


    ${CMAKE_SOURCE_DIR}/include/
    ${CMAKE_SOURCE_DIR}/include/commMsgQueue/
    ${CMAKE_SOURCE_DIR}/include/modbus/
    ${CMAKE_SOURCE_DIR}/include/nanomsg/
    ${CMAKE_SOURCE_DIR}/src/proto/obstacle/

    ${CMAKE_SOURCE_DIR}/src/
    ${CMAKE_SOURCE_DIR}/src/adrc/
    ${CMAKE_SOURCE_DIR}/src/boat/
    ${CMAKE_SOURCE_DIR}/src/common/
    ${CMAKE_SOURCE_DIR}/src/conf/
    ${CMAKE_SOURCE_DIR}/src/control/
    ${CMAKE_SOURCE_DIR}/src/filter/
    ${CMAKE_SOURCE_DIR}/src/hal/
    ${CMAKE_SOURCE_DIR}/src/math/
    ${CMAKE_SOURCE_DIR}/src/mrac/
    ${CMAKE_SOURCE_DIR}/src/pid/
    ${CMAKE_SOURCE_DIR}/src/radar/
    ${CMAKE_SOURCE_DIR}/src/user_time/
    ${CMAKE_SOURCE_DIR}/src/vehicle_state/
    ${CMAKE_SOURCE_DIR}/src/simulation/
    ${CMAKE_SOURCE_DIR}/src/util/
    ${CMAKE_SOURCE_DIR}/src/xml/

    ${CMAKE_SOURCE_DIR}/src/planning/
)

link_directories(
  ${CMAKE_SOURCE_DIR}/3rdLibs/libiconv/lib/
	${CMAKE_SOURCE_DIR}/3rdLibs/nanomsg/lib/
  ${CMAKE_SOURCE_DIR}/3rdLibs/protobuf/lib/
  ${CMAKE_SOURCE_DIR}/3rdLibs/modbus/lib/
  ${CMAKE_SOURCE_DIR}/3rdLibs/zmq/lib/
  ${CMAKE_SOURCE_DIR}/3rdLibs/xml/lib/
)

link_libraries(
  nanomsg
  protobuf
  modbus
  iconv
  zmq
  rt
  pthread
  m
  socketcan
  mxml
)


message("++++++++++++++Start Build+++++++++++++++++")

aux_source_directory(./src SRC_LIST)
aux_source_directory(./src/proto/obstacle OBS_SRC_LIST)

aux_source_directory(./src/adrc SRC_adrc)
aux_source_directory(./src/boat SRC_boat)
aux_source_directory(./src/conf SRC_conf)
aux_source_directory(./src/common SRC_common)
aux_source_directory(./src/control SRC_control)
aux_source_directory(./src/filter SRC_filter)
aux_source_directory(./src/hal SRC_hal)
aux_source_directory(./src/math SRC_math)
aux_source_directory(./src/mrac SRC_mrac)
aux_source_directory(./src/pid SRC_pid)
aux_source_directory(./src/radar SRC_radar)
aux_source_directory(./src/user_time SRC_time)
aux_source_directory(./src/vehicle_state SRC_vehicle_state)
aux_source_directory(./src/simulation SRC_simulation)
aux_source_directory(./src/xml SRC_xml)
aux_source_directory(./src/util SRC_util)
aux_source_directory(./src/planning SRC_planner)

#编译
add_executable(${PROJECT_NAME} ${SRC_LIST} ${OBS_SRC_LIST} 
               ${SRC_adrc} 
               ${SRC_boat} 
               ${SRC_conf} 
               ${SRC_common} 
               ${SRC_control}
               ${SRC_filter}
               ${SRC_hal}
               ${SRC_math}
               ${SRC_mrac} 
               ${SRC_pid}
               ${SRC_radar}
               ${SRC_time} 
               ${SRC_vehicle_state}
               ${SRC_simulation} 
               ${SRC_xml}
               ${SRC_util}
               ${SRC_planner})

#链接
target_link_libraries(${PROJECT_NAME} 
  nanomsg
  protobuf
  modbus
  iconv
  zmq
  rt
  pthread
  m
  socketcan
  mxml
)

