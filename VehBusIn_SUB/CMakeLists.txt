# cmake_minimum_required(VERSION 3.16)

# project(libVehBusIn_SUB LANGUAGES C CXX)


# set(CMAKE_C_COMPILER /usr/local/ARM-toolchain/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc)
# set(CMAKE_CXX_COMPILER /usr/local/ARM-toolchain/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++)


# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MOS_LIB /home/c/MAXEYE/ADCU/dds_mos/linux-ax750)
set(CMAKE_PREFIX_PATH ${MOS_LIB})
set(CMAKE_BUILD_TYPE Release)
set(libVehBusIn libVehBusIn_SUB)

# 设置可执行文件的输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


find_package(met_pack REQUIRED)
# find_package(nlohmann_json REQUIRED)
if(NOT TARGET communication)
  find_package(communication QUIET)
endif()

add_executable(${libVehBusIn}
	/home/c/MAXEYE/ADCU/dds_mos/Src/VehBusIn_SUB/AdApter_VehBusIn_sub.cpp  )
  #/home/c/MAXEYE/ADCU/dds_mos/Feature_Acore_Release/requirements/headerfile/IPC_Pub_SetAndPrintValue.cpp) #add_library


target_link_libraries(${libVehBusIn} PUBLIC mos::met_pack rt pthread mos::communication)


target_include_directories(
    ${libVehBusIn} 
    PUBLIC 
    /home/c/MAXEYE/ADCU/dds_mos/linux-ax750/include
    /home/c/MAXEYE/ADCU/dds_mos/Feature_Acore_Release/requirements/headerfile
    /home/c/MAXEYE/ADCU/dds_mos/Src/VehBusIn_SUB
    /home/c/ExternalLib/json/include
    )

# add_subdirectory()
