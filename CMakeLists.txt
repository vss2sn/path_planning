project(path_planning)

cmake_minimum_required(VERSION 3.5)
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

# For clang-tidy
# string(CONCAT CLANG_TIDY_CHECKS
#   "*,"                                    # enable all1
#   "-modernize-use-trailing-return-type,"  # style-choice
#   "-fuchsia-default-arguments-calls,"     # allow use of default values in constructors
#   "-cert-msc50-cpp,"                      # rand() is good enough for the current usage
#   "-cert-msc30-c,"                        # rand() is good enough for the current usage
#   "-fuchsia-overloaded-operator"          # +, -, () overloaded for node
# )
# set( CMAKE_CXX_CLANG_TIDY "clang-tidy;-checks=${CLANG_TIDY_CHECKS}" )

set( CMAKE_CXX_STANDARD 17 )
# set( CMAKE_CXX_FLAGS "${CXX_FLAGS} -pthread -Wall -Werror -O3" )

# TODO(vss2sn): Remove glob
file( GLOB LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp )
list( REMOVE_ITEM LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp )
file( GLOB LIB_TESTS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp )

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

add_subdirectory(lib)

if(BUILD_INDIVIDUAL)

  add_definitions(-DBUILD_INDIVIDUAL)
  foreach(test_source_file ${LIB_SOURCES})
    get_filename_component(test_output_file ${test_source_file} NAME_WE)
    add_executable( ${test_output_file} ${test_source_file})
    target_include_directories(${test_output_file} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    set_target_properties( ${test_output_file} PROPERTIES CXX_FLAGS "${CXX_FLAGS}  -pthread -Wall -Werror -O3 -g" )
    target_link_libraries( ${test_output_file} utils )
  endforeach(test_source_file ${LIB_SOURCES})

else (BUILD_INDIVIDUAL)

  add_library(algos)
  target_include_directories(algos PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_sources(algos PRIVATE ${LIB_SOURCES})
  target_link_libraries(algos PUBLIC utils)

  add_executable( main ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp )
  target_link_libraries( main PRIVATE utils algos)

  if(CHECK_COVERAGE)
    target_compile_options(algos PUBLIC -O0 -g --coverage)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.13)
      target_link_options(algos  PUBLIC --coverage)
    else()
    target_link_libraries(algos PUBLIC --coverage)
    endif()
  else(CHECK_COVERAGE)
    set_target_properties(main algos utils PROPERTIES CXX_FLAGS "${CXX_FLAGS}  -pthread -Wall -Werror -O0 -g")
  endif(CHECK_COVERAGE)


  if (RUN_TESTS)
    find_package(GTest)  # For GTest
    if(GTest_FOUND)  # GTestInstalled
      include_directories(${GTEST_INCLUDE_DIRS})
    else(GTest_FOUND)  # Using GTEST as submodule
      include(gtest.cmake)
    endif(GTest_FOUND)
    enable_testing()
    add_subdirectory(tests)
  endif(RUN_TESTS)

endif (BUILD_INDIVIDUAL)
