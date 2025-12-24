find_package(CycloneDDS-CXX REQUIRED)

find_package(argparse REQUIRED)
if(NOT TARGET communication)
  find_package(communication QUIET)
  find_package(nlohmann_json REQUIRED)
endif()
find_package(met_pack REQUIRED)
add_executable(${libLDP_Obj}
${CMAKE_CURRENT_LIST_DIR}/AdApter_LDP_Obj.cpp 
${CMAKE_CURRENT_LIST_DIR}/AdApter_LDP_Obj.h 
${BEV_VO_Topic}/LDP_Obj.h
${DDS_MSG}/LDP_Obj.hpp
${DDS_MSG}/LDP_Obj.cpp) #add_library

if(UNIX)
  target_link_libraries(${libLDP_Obj} PUBLIC CycloneDDS-CXX::ddscxx mos::met_pack rt pthread mos::communication argparse::argparse)
else()
  if(NOT TARGET communication)
    target_link_libraries(${libLDP_Obj} PUBLIC CycloneDDS-CXX::ddscxx mos::met_pack mswsock dbghelp mos::communication argparse::argparse)
  else()
    target_link_libraries(${libLDP_Obj} PUBLIC CycloneDDS-CXX::ddscxx mos::met_pack mswsock dbghelp communication argparse::argparse)
  endif()
endif()

target_include_directories(
    ${libLDP_Obj} 
    PUBLIC 
    ${MOS_LIB}/include
    ${BEV_VO_Topic}
    ${dds_install}/include/ddscxx
    ${dds_install}/include
    ${DDS_MSG}
    )
