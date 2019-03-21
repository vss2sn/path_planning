project(path_planning)

cmake_minimum_required(VERSION 3.5)

add_definitions(-std=c++11)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(White       "${Esc}[37m")
endif()

set( CXX_FLAGS "-w" )
set( CMAKE_CXX_FLAGS "${CXX_FLAGS} -std=c++11 -pthread" )

include_directories( ${CMAKE_SOURCE_DIR}/inc )

file( GLOB LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp )
file( GLOB LIB_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h )
file( GLOB LIB_TESTS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp )

# Building utils.cpp as  a library as common to multiple files
add_library( utils SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp ${LIB_HEADERS} )
list( REMOVE_ITEM LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp )

# If building each file with it's own output, set to ON
# If buiding single file (main.cpp) to test any/all protocols, set to OFF

list(REMOVE_ITEM LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

option( BUILD_INDIVIDUAL "BUILD_INDIVIDUAL" OFF)
option( TEST "TEST" ON)

if(BUILD_INDIVIDUAL)

  add_definitions(-DBUILD_INDIVIDUAL)
  message("${Blue} Individual files being built ${ColourReset}")
  foreach(test_source_file ${LIB_SOURCES})
    string( REPLACE ".cpp" "" test_output_file ${test_source_file} )
    string( REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/src/" "" test_output_file ${test_output_file} )
    message( ${test_output_file} )
    add_executable( ${test_output_file} ${test_source_file} ${LIB_HEADERS})
    target_link_libraries( ${test_output_file} utils )
  endforeach(test_source_file ${LIB_SOURCES})
  message( "${Blue}End of list ${ColourReset}" )

else (BUILD_INDIVIDUAL)

  add_library( algos SHARED ${LIB_SOURCES} ${LIB_HEADERS} )
  add_executable( main ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp ${LIB_HEADERS} )
  target_link_libraries( main utils algos)

  if (TEST)
    find_package(GTest REQUIRED)
    enable_testing()
    add_definitions(-DTEST)
    include_directories(${GTEST_INCLUDE_DIRS})
    foreach(test_source_file ${LIB_TESTS_SOURCES})
      string( REPLACE ".cpp" "" test_output_file ${test_source_file} )
      string( REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/tests/" "" test_output_file ${test_output_file} )
      add_executable( ${test_output_file} ${test_source_file} ${LIB_HEADERS})
      target_link_libraries( ${test_output_file} ${GTEST_LIBRARIES} pthread utils algos )
      add_test(${test_output_file} ${test_output_file})
      add_dependencies(main ${test_output_file}) # ensure main is built after the test grids.
    endforeach(test_source_file ${LIB_SOURCES})

    add_custom_command(
     TARGET main # Ensures tests run only after main
     POST_BUILD
     COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> -R "^${UNIT_TEST}$" --output-on-failures
    )
  endif (TEST)

endif (BUILD_INDIVIDUAL)
