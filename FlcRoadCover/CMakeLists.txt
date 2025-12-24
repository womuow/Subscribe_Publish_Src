find_package(CycloneDDS-CXX REQUIRED)

find_package(argparse REQUIRED)
if(NOT TARGET communication)
  find_package(communication QUIET)
  find_package(nlohmann_json REQUIRED)
endif()
find_package(met_pack REQUIRED)
add_executable(${libFlcRoadCover}
${CMAKE_CURRENT_LIST_DIR}/AdApter_FlcRoadCover.cpp 
${CMAKE_CURRENT_LIST_DIR}/AdApter_FlcRoadCover.h 
${BEV_VO_Topic}/FlcRoadCover.h
${DDS_MSG}/FlcRoadCover.hpp
${DDS_MSG}/FlcRoadCover.cpp) #add_library

if(UNIX)
  target_link_libraries(${libFlcRoadCover} PUBLIC CycloneDDS-CXX::ddscxx mos::met_pack rt pthread mos::communication argparse::argparse)
else()
  if(NOT TARGET communication)
    target_link_libraries(${libFlcRoadCover} PUBLIC CycloneDDS-CXX::ddscxx mos::met_pack mswsock dbghelp mos::communication argparse::argparse)
  else()
    target_link_libraries(${libFlcRoadCover} PUBLIC CycloneDDS-CXX::ddscxx mos::met_pack mswsock dbghelp communication argparse::argparse)
  endif()
endif()

target_include_directories(
    ${libFlcRoadCover} 
    PUBLIC 
    ${MOS_LIB}/include
    ${BEV_VO_Topic}
    ${dds_install}/include/ddscxx
    ${dds_install}/include
    ${DDS_MSG}
    )
