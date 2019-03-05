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
set( CMAKE_CXX_FLAGS "${CXX_FLAGS} -std=c++11 -pthread -fpermissive" )

include_directories( ${CMAKE_SOURCE_DIR}/inc )

file( GLOB LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp )
file( GLOB LIB_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h )

# Building utils.cpp as  a library as common to multiple files
add_library( utils ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp ${LIB_HEADERS} )
list( REMOVE_ITEM LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp )

# If building each file with it's own output, set to ON
# If buiding single file (main.cpp) to test any/all protocols, set to OFF

option( BUILD_INDIVIDUAL "BUILD_INDIVIDUAL" OFF)
set(file_list ${LIB_SOURCES})
if(BUILD_INDIVIDUAL)

  add_definitions(-DBUILD_INDIVIDUAL)
  list(REMOVE_ITEM file_list ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
  message("${Green}Individual files being built")
  foreach(test_source_file ${file_list})
    string( REPLACE ".cpp" "" test_output_file ${test_source_file} )
    string( REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/src/" "" test_output_file ${test_output_file} )
    message( ${test_output_file} )
    add_executable( ${test_output_file} ${test_source_file} ${LIB_HEADERS})
    target_link_libraries( ${test_output_file} utils )
  endforeach(test_source_file ${LIB_SOURCES})
  message( "${Green}End of list ${ColourReset}" )
else (BUILD_INDIVIDUAL)
  message( main.cpp )
  #add_executable( main.o ${LIB_SOURCES} ${LIB_HEADERS} )
  add_executable( main ${LIB_SOURCES} ${LIB_HEADERS} )
  target_link_libraries( main utils)
endif (BUILD_INDIVIDUAL)
