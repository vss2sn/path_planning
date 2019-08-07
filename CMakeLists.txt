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
file( GLOB LIB_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp )
file( GLOB LIB_TESTS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp )

# Building utils.cpp as  a library as common to multiple files
add_library( utils SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp ${LIB_HEADERS} )
add_library( test_utils SHARED ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_utils.cpp ${LIB_HEADERS} )

list( REMOVE_ITEM LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp )
list( REMOVE_ITEM LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp )
list( REMOVE_ITEM LIB_TESTS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_utils.cpp )

# If building each file with it's own output, set to ON
# If buiding single file (main.cpp) to test any/all protocols, set to OFF
option( BUILD_INDIVIDUAL "BUILD_INDIVIDUAL" OFF)
option( TEST "TEST" ON)
option( CHECK_COVERAGE "CHECK_COVERAGE" ON)
option( DYNAMIC_ALGOS "DYNAMIC_ALGOS" ON)
option( CUSTOM_DEBUG_HELPER_FUNCION "CUSTOM_DEBUG_HELPER_FUNCION" OFF)
option( TRAVIS "CONFIG_FOR_TRAVIS" OFF)

# Ensures dynamic runs for individual files allowed
if(BUILD_INDIVIDUAL)
  option( DYNAMIC_ALGOS ON)
  option( CUSTOM_DEBUG_HELPER_FUNCION ON)
  option( CHECK_COVERAGE OFF)
endif(BUILD_INDIVIDUAL)

# No way to check for dynamic algos as they have random obstacle generation
# Hence removing from code coverage tests
if(CHECK_COVERAGE)
  option( DYNAMIC_ALGOS OFF)
  option( CUSTOM_DEBUG_HELPER_FUNCION OFF)
endif(CHECK_COVERAGE)

if(CUSTOM_DEBUG_HELPER_FUNCION)
  add_definitions(-DCUSTOM_DEBUG_HELPER_FUNCION)
endif(CUSTOM_DEBUG_HELPER_FUNCION)

if(DYNAMIC_ALGOS)
  add_definitions(-D DYNAMIC_ALGOS)
endif(DYNAMIC_ALGOS)

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

    # For coverage check
    if(CHECK_COVERAGE)
      LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
      include(CodeCoverage)
      APPEND_COVERAGE_COMPILER_FLAGS()
    endif(CHECK_COVERAGE)

    # For GTEST
    if(TRAVIS)
      # Using GTEST as submodule
      enable_testing()
      add_definitions(-DTEST)
      include(gtest.cmake)
    else(TRAVIS)
      # Installed
      find_package(GTest REQUIRED)
      enable_testing()
      add_definitions(-DTEST)
      include_directories(${GTEST_INCLUDE_DIRS})
    endif(TRAVIS)

    foreach(test_source_file ${LIB_TESTS_SOURCES})
      string( REPLACE ".cpp" "" test_output_file ${test_source_file} )
      string( REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/tests/" "" test_output_file ${test_output_file} )
      add_executable( ${test_output_file} ${test_source_file} ${LIB_HEADERS})
      target_link_libraries( ${test_output_file} ${GTEST_LIBRARIES} pthread utils algos test_utils gtest)
      add_test(${test_output_file} ${test_output_file})

      if(CHECK_COVERAGE)
        set(COVERAGE_LCOV_EXCLUDES '${CMAKE_CURRENT_SOURCE_DIR}/tests/*' # Will skew results
                                   '/usr/*' # Gtest
                                   '${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp' # Each tet fle is the equivalent of main
                                   '${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp' # Only util functions that serve as helpers
                                   '${CMAKE_CURRENT_SOURCE_DIR}/src/rrt.cpp' # No test
                                   '${CMAKE_CURRENT_SOURCE_DIR}/inc/rrt.hpp') # No test
        SETUP_TARGET_FOR_COVERAGE_LCOV(NAME ${test_output_file}_coverage EXECUTABLE ${test_output_file} )
        target_link_libraries(${test_output_file} gcov)
        add_custom_command(
          TARGET main # Ensures tests run only after main
          POST_BUILD
          COMMAND make ${test_output_file}_coverage WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/)
      endif(CHECK_COVERAGE)

      add_dependencies(main ${test_output_file}) # ensure main is built after the test grids.

    endforeach(test_source_file ${LIB_SOURCES})

    # No need to run tests separately if check coverage is being run as that will run the tests anyway
    if(NOT CHECK_COVERAGE)
      add_custom_command(
       TARGET main # Ensures tests run only after main
       POST_BUILD
       COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIGURATION> -R "^${UNIT_TEST}$" --output-on-failures)
    endif(NOT CHECK_COVERAGE)

  endif (TEST)

endif (BUILD_INDIVIDUAL)
