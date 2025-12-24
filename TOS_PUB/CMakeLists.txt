cmake_minimum_required(VERSION 3.16)

project(TOSPUB LANGUAGES C CXX)


set(CMAKE_C_COMPILER /usr/local/ARM-toolchain/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/local/ARM-toolchain/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MOS_LIB /home/c/MAXEYE/ADCU/dds_mos/linux-ax750)
set(CMAKE_PREFIX_PATH ${MOS_LIB})
set(CMAKE_BUILD_TYPE Release)
set(TOSPUB TOSPUB)
find_package(met_pack REQUIRED)
# find_package(nlohmann_json REQUIRED)
if(NOT TARGET communication)
  find_package(communication QUIET)
endif()

add_executable(${TOSPUB}
/home/c/MAXEYE/ADCU/dds_mos/Src/TOS_PUB/AdApter_TOS_PUB.cpp 
/home/c/MAXEYE/ADCU/dds_mos/Src/TOS_PUB/AdApter_TOS_PUB.h 
/home/c/MAXEYE/ADCU/dds_mos/Src/TOS_PUB/IPC_Pub.h) #add_library


 target_link_libraries(${TOSPUB} PUBLIC mos::met_pack rt pthread mos::communication)


target_include_directories(
    ${TOSPUB} 
    PUBLIC 
    /home/c/MAXEYE/ADCU/dds_mos/linux-ax750/include
    /home/c/MAXEYE/ADCU/dds_mos/Src/TOS_PUB
    #/home/c/MAXEYE/ADCU/dds_mos
    /home/c/MAXEYE/ADCU/dds_mos/Src/TOS_PUB/json/include
    )

# add_subdirectory()