# cmake_minimum_required(VERSION 3.16)

# project(libSWP1_SUB LANGUAGES C CXX)


# set(CMAKE_C_COMPILER /usr/local/ARM-toolchain/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc)
# set(CMAKE_CXX_COMPILER /usr/local/ARM-toolchain/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++)


# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MOS_LIB /home/c/MAXEYE/ADCU/dds_mos/linux-ax750)
set(CMAKE_PREFIX_PATH ${MOS_LIB})
set(CMAKE_BUILD_TYPE Release)
set(libSWP1 libSWP1_SUB)

# 设置可执行文件的输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


find_package(met_pack REQUIRED)
# find_package(nlohmann_json REQUIRED)
if(NOT TARGET communication)
  find_package(communication QUIET)
endif()


add_definitions(-DMET_SOC_SWP1_PARAM_H=1)

add_executable(${libSWP1}
  /home/c/MAXEYE/ADCU/dds_mos/Src/Maxieye_C_headfile/MET_SOC_ALLMapPrint.cpp
	/home/c/MAXEYE/ADCU/dds_mos/Src/MET_SOC_SWP1_SUB/Adapter_SWP1_sub.cpp ) #add_library


target_link_libraries(${libSWP1} PUBLIC mos::met_pack rt pthread mos::communication)


target_include_directories(
    ${libSWP1} 
    PUBLIC 
    /home/c/MAXEYE/ADCU/dds_mos/linux-ax750/include
    /home/c/MAXEYE/ADCU/dds_mos/Src/Maxieye_C_headfile
    /home/c/MAXEYE/ADCU/dds_mos/Src/MET_SOC_SWP1_SUB
    /home/c/ExternalLib/json/include
    )

# add_subdirectory()