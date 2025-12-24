find_package(CycloneDDS-CXX REQUIRED)

find_package(argparse REQUIRED)
if(NOT TARGET communication)
  find_package(communication QUIET)
  find_package(nlohmann_json REQUIRED)
endif()
find_package(met_pack REQUIRED)
add_executable(${libFusedFrontObject}
${CMAKE_CURRENT_LIST_DIR}/AdApter_FusedFrontObject.cpp 
${CMAKE_CURRENT_LIST_DIR}/AdApter_FusedFrontObject.h 
${BEV_VO_Topic}/FusedFrontObject.h
${DDS_MSG}/FusedFrontObject.hpp
${DDS_MSG}/FusedFrontObject.cpp) #add_library

if(UNIX)
  target_link_libraries(${libFusedFrontObject} PUBLIC CycloneDDS-CXX::ddscxx mos::met_pack rt pthread mos::communication argparse::argparse)
else()
  if(NOT TARGET communication)
    target_link_libraries(${libFusedFrontObject} PUBLIC CycloneDDS-CXX::ddscxx mos::met_pack mswsock dbghelp mos::communication argparse::argparse)
  else()
    target_link_libraries(${libFusedFrontObject} PUBLIC CycloneDDS-CXX::ddscxx mos::met_pack mswsock dbghelp communication argparse::argparse)
  endif()
endif()

target_include_directories(
    ${libFusedFrontObject} 
    PUBLIC 
    ${MOS_LIB}/include
    ${BEV_VO_Topic}
    ${dds_install}/include/ddscxx
    ${dds_install}/include
    ${DDS_MSG}
    )
