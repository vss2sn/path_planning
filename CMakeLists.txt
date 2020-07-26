project(path_planning)

cmake_minimum_required(VERSION 3.16)
include(CMakeDependentOption)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(White       "${Esc}[37m")
endif()

# string(CONCAT CLANG_TIDY_CHECKS
#   "*,"                                    # enable all1
#   "-modernize-use-trailing-return-type,"  # style-choice
#   "-fuchsia-default-arguments-calls,"     # allow use of default values in constructors
#   "-cert-msc50-cpp,"                      # rand() is good enough for the current usage
#   "-cert-msc30-c,"                        # rand() is good enough for the current usage
#   "-fuchsia-overloaded-operator"          # +, -, () overloaded for node
# )

set( CMAKE_CXX_STANDARD 17 )
# set( CMAKE_CXX_FLAGS "${CXX_FLAGS} -pthread -Wall -Werror -O3" )
# set( CMAKE_CXX_CLANG_TIDY "clang-tidy;-checks=${CLANG_TIDY_CHECKS}" )

include_directories( ${CMAKE_SOURCE_DIR}/inc )

file( GLOB LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp )
file( GLOB LIB_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.hpp )
file( GLOB LIB_TESTS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp )

# Building utils.cpp as  a library as common to multiple files
add_library( utils SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp ${LIB_HEADERS} )

list( REMOVE_ITEM LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp )
list( REMOVE_ITEM LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp )

# If building each file with it's own output, set to ON
# If buiding single file (main.cpp) to test any/all protocols, set to OFF
option( BUILD_INDIVIDUAL "Build each algorithm .cpp file with its own executable" OFF)
# default OFF
cmake_dependent_option( RUN_TESTS "Build and run tests" OFF "NOT BUILD_INDIVIDUAL" OFF)
# default OFF, change by user input if and only if condition allows (BUILD_INDIVIDUAL=OFF)
cmake_dependent_option( CHECK_COVERAGE "Run code coverage check" OFF "RUN_TESTS" OFF)
# default OFF, change by user input if and only if condition allows (RUN_TESTS=ON)
cmake_dependent_option( DYNAMIC_ALGOS "Build and run algorithms with obstacle detection in real time (D* Lite)" ON  "NOT RUN_TESTS" OFF)
# default OFF, change by user input if and only if condition allows (RUN_TESTS=OFF)
cmake_dependent_option( CUSTOM_DEBUG_HELPER_FUNCION "Build custom debug helper functions" OFF "NOT RUN_TESTS" OFF)
# default OFF, change by user input if and only if condition allows (RUN_TESTS=OFF)

if(CUSTOM_DEBUG_HELPER_FUNCION)
  add_definitions(-DCUSTOM_DEBUG_HELPER_FUNCION)
endif(CUSTOM_DEBUG_HELPER_FUNCION)

if(DYNAMIC_ALGOS)
  add_definitions(-DDYNAMIC_ALGOS)
endif(DYNAMIC_ALGOS)

if(BUILD_INDIVIDUAL)

  add_definitions(-DBUILD_INDIVIDUAL)
  message("${Blue} Individual files being built ${ColourReset}")
  foreach(test_source_file ${LIB_SOURCES})
    get_filename_component(test_output_file ${test_source_file} NAME_WE)
    message( ${test_output_file} )
    add_executable( ${test_output_file} ${test_source_file} ${LIB_HEADERS})
    set_target_properties( ${test_output_file} PROPERTIES CXX_FLAGS "${CXX_FLAGS}  -pthread -Wall -Werror -O3 -g" )
    target_link_libraries( ${test_output_file} utils )
  endforeach(test_source_file ${LIB_SOURCES})
  message( "${Blue}End of list ${ColourReset}" )

else (BUILD_INDIVIDUAL)

  add_library( algos SHARED ${LIB_SOURCES} ${LIB_HEADERS} )
  add_executable( main ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp ${LIB_HEADERS} )
  target_link_libraries( main utils algos)

  if (RUN_TESTS)
    enable_testing()
    add_subdirectory(tests)
  endif(RUN_TESTS)

  if(CHECK_COVERAGE)
    set_target_properties( main algos utils PROPERTIES CXX_FLAGS "${CXX_FLAGS} -pthread -Wall -Werror --coverage -O0 -g" )
  else(CHECK_COVERAGE)
    set_target_properties( main algos utils PROPERTIES CXX_FLAGS "${CXX_FLAGS} -pthread -Wall -Werror -O3")
  endif(CHECK_COVERAGE)

endif (BUILD_INDIVIDUAL)
