# cmake_minimum_required(VERSION 3.16)

# project(libFuncTgtVisnID_SUB LANGUAGES C CXX)


# set(CMAKE_C_COMPILER /usr/local/ARM-toolchain/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc)
# set(CMAKE_CXX_COMPILER /usr/local/ARM-toolchain/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++)


# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MOS_LIB ${CMAKE_SOURCE_DIR}/linux-ax750)
set(CMAKE_PREFIX_PATH ${MOS_LIB})
set(CMAKE_BUILD_TYPE Release)
set(libFuncTgtVisnID libFuncTgtVisnID_SUB)

# 设置可执行文件的输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/MOS_SUB)


find_package(met_pack REQUIRED)
# find_package(nlohmann_json REQUIRED)
if(NOT TARGET communication)
  find_package(communication QUIET)
endif()



add_definitions(-DIDT_FUNCTGTVISNID_H=1)

add_executable(${libFuncTgtVisnID}
  ${CMAKE_SOURCE_DIR}/Src/Feature_Acore_Release/requirements/headerfile/Feature_ALLSetPrint.cpp  
	${CMAKE_SOURCE_DIR}/Src/FuncTgtVisnID_SUB/AdApter_FuncTgtVisnID_sub.cpp )
  #${CMAKE_SOURCE_DIR}/Src/Feature_Acore_Release/requirements/headerfile/IPC_Pub_SetAndPrintValue.cpp) #add_library


target_link_libraries(${libFuncTgtVisnID} PUBLIC mos::met_pack rt pthread mos::communication)


target_include_directories(
    ${libFuncTgtVisnID} 
    PUBLIC 
    ${CMAKE_SOURCE_DIR}/linux-ax750/include
    ${CMAKE_SOURCE_DIR}/Src/Feature_Acore_Release/requirements/headerfile
    ${CMAKE_SOURCE_DIR}/Src/FuncTgtVisnID_SUB
    /home/c/ExternalLib/json/include
    )
    
# message(STATUS "CMAKE_SOURCE_DIR: 222${CMAKE_SOURCE_DIR}")
# add_subdirectory()
