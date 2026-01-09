# cmake_minimum_required(VERSION 3.16)



# set(CMAKE_C_COMPILER /usr/local/ARM-toolchain/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc)
# set(CMAKE_CXX_COMPILER /usr/local/ARM-toolchain/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++)

# project(libLongCtrlObjInfoPUB LANGUAGES C CXX)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MOS_LIB ${CMAKE_SOURCE_DIR}/linux-ax750)
set(CMAKE_PREFIX_PATH ${MOS_LIB})
set(CMAKE_BUILD_TYPE Release)
set(libLongCtrlObjInfoPUB libLongCtrlObjInfo_PUB)


# 设置可执行文件的输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


find_package(met_pack REQUIRED)
# find_package(nlohmann_json REQUIRED)
if(NOT TARGET communication)
  find_package(communication QUIET)
endif()

add_executable(${libLongCtrlObjInfoPUB}
${CMAKE_SOURCE_DIR}/Src/LongCtrlObjInfo_PUB/AdApter_LongCtrlObjInfo_PUB.cpp 
) #add_library


 target_link_libraries(${libLongCtrlObjInfoPUB} PUBLIC mos::met_pack rt pthread mos::communication)


target_include_directories(
    ${libLongCtrlObjInfoPUB} 
    PUBLIC 
    ${CMAKE_SOURCE_DIR}/linux-ax750/include
    ${CMAKE_SOURCE_DIR}/Src/Feature_Acore_Release/requirements/headerfile
    ${CMAKE_SOURCE_DIR}/Src/LongCtrlObjInfo_PUB
    /home/c/ExternalLib/json/include
    )

# add_subdirectory()