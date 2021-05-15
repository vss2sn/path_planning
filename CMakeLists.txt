set(CMAKE_C_COMPILER "/usr/bin/gcc-10")
set(CMAKE_CXX_COMPILER "/usr/bin/g++-10")

project(path_planning)

cmake_minimum_required(VERSION 3.15)
include(CMakeDependentOption)

# enable repository specific options
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/RepositorySpecificOptions.cmake")

# enable cache system
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Cache.cmake")

set(PATH_TO_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/main/main.cpp")
file( GLOB LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp )
file( GLOB LIB_TESTS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp )


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
    add_executable( ${test_output_file})
    target_sources(${test_output_file} PRIVATE ${test_source_file})
    target_include_directories(${test_output_file} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    set_target_properties(${test_output_file}
      PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
        CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wall -Werror -g"
    )
    target_link_libraries( ${test_output_file} utils )
  endforeach(test_source_file ${LIB_SOURCES})

else (BUILD_INDIVIDUAL)

  add_library(algos)
  target_include_directories(algos PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_sources(algos PRIVATE ${LIB_SOURCES})
  target_link_libraries(algos PUBLIC utils)
  set_target_properties(algos
    PROPERTIES
      CXX_STANDARD 20
      CXX_STANDARD_REQUIRED YES
      CXX_EXTENSIONS NO
  )

  add_executable( main ${PATH_TO_MAIN} )
  target_link_libraries( main PRIVATE utils algos)

  if(CHECK_COVERAGE)
    target_compile_options(algos PUBLIC -O0 -g --coverage)
    target_link_options(algos  PUBLIC --coverage)
  else(CHECK_COVERAGE)
    set_target_properties(algos
      PROPERTIES
        CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wall -Werror -g"
    )
    set_target_properties( main
      PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
        CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wall -Werror -g"
    )
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
