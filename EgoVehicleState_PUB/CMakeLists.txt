# cmake_minimum_required(VERSION 3.16)



# set(CMAKE_C_COMPILER /usr/local/ARM-toolchain/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc)
# set(CMAKE_CXX_COMPILER /usr/local/ARM-toolchain/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-g++)

# project(libEgoVehicleStatePUB LANGUAGES C CXX)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MOS_LIB /home/c/MAXEYE/ADCU/dds_mos/linux-ax750)
set(CMAKE_PREFIX_PATH ${MOS_LIB})
set(CMAKE_BUILD_TYPE Release)
set(libEgoVehicleStatePUB libEgoVehicleState_PUB)

# 设置可执行文件的输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


find_package(met_pack REQUIRED)
# find_package(nlohmann_json REQUIRED)
if(NOT TARGET communication)
  find_package(communication QUIET)
endif()

add_executable(${libEgoVehicleStatePUB}
/home/c/MAXEYE/ADCU/dds_mos/Src/EgoVehicleState_PUB/AdApter_EgoVehicleState_PUB.cpp 
) #add_library


 target_link_libraries(${libEgoVehicleStatePUB} PUBLIC mos::met_pack rt pthread mos::communication)


target_include_directories(
    ${libEgoVehicleStatePUB} 
    PUBLIC 
    /home/c/MAXEYE/ADCU/dds_mos/linux-ax750/include
    /home/c/MAXEYE/ADCU/dds_mos/Feature_Acore_Release/requirements/headerfile
    /home/c/MAXEYE/ADCU/dds_mos/Src/EgoVehicleState_PUB
    /home/c/ExternalLib/json/include
    )

# add_subdirectory()