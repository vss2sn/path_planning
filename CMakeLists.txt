project(path_planning)

cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++11)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${ESC}[m]")
  set(Red         "${ESC}[31]")
  set(Green       "${ESC}[32]")
  set(Yellow      "${ESC}[33]")
  set(Blue        "${ESC}[34]")
  set(White       "${Esc}[37m")
endif()

set( CXX_FLAGS "-w" )
set( CMAKE_CXX_FLAGS "${CXX_FLAGS} -std=c++11 -pthread" )

include_directories( ${CMAKE_SOURCE_DIR}/inc )

file( GLOB LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp )
file( GLOB LIB_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h )

message("${Green}Individual files being built")
foreach(test_source_file ${LIB_SOURCES})
  string( REPLACE ".cpp" ".o" test_output_file ${test_source_file} )
  string( REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/src/" "" test_output_file ${test_output_file} )
  message( ${test_output_file} )
  add_executable( ${test_output_file} ${test_source_file} ${LIB_HEADERS} )
endforeach(test_source_file ${LIB_SOURCES})
message( "${Green}End of list ${ColourReset}" )
