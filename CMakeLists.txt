cmake_minimum_required(VERSION 3.15)
project(path_planning CXX)

include(cmake/StandardProjectSettings.cmake)
include(cmake/PreventInSourceBuilds.cmake)

# Link this 'library' to set the c++ standard / compile-time options requested
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
  option(ENABLE_BUILD_WITH_TIME_TRACE "Enable -ftime-trace to generate time tracing .json files on clang" OFF)
  if(ENABLE_BUILD_WITH_TIME_TRACE)
    target_compile_options(project_options INTERFACE -ftime-trace)
  endif()
endif()

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

# enable cache system
include(cmake/Cache.cmake)

# standard compiler warnings
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

# sanitizer options if supported by compiler
include(cmake/Sanitizers.cmake)
enable_sanitizers(project_options)

# allow for static analysis options
include(cmake/StaticAnalyzers.cmake)

# enable repository specific options
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/ProjectSpecificOptions.cmake")

# enable cache system
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Cache.cmake")

set(PATH_TO_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/main/main.cpp")
file( GLOB LIB_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/a_star.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/dijkstra.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/rrt_star.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/rrt.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lpa_star.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/d_star_lite.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ant_colony.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/genetic_algorithm.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/jump_point_search.cpp
)

add_subdirectory(lib)

if (BUILD_INDIVIDUAL)
  add_definitions(-DBUILD_INDIVIDUAL)
  foreach(file ${LIB_SOURCES})
    get_filename_component(ouput_file ${file} NAME_WE)
    add_executable(${ouput_file})
    target_include_directories(${ouput_file} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_sources(${ouput_file} PRIVATE ${file})
    target_compile_options (${ouput_file}  PRIVATE -O3)
    target_link_libraries(${ouput_file} PRIVATE utils project_options project_warnings)
  endforeach(file ${LIB_SOURCES})
else (BUILD_INDIVIDUAL)
  add_library(algos)
  target_include_directories(algos PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_sources(algos PRIVATE ${LIB_SOURCES})
  target_link_libraries(algos PUBLIC utils PRIVATE project_options project_warnings)
  add_executable(main ${CMAKE_CURRENT_SOURCE_DIR}/main/main.cpp)
  target_compile_options (main  PRIVATE -O3)
  target_link_libraries(main PRIVATE utils algos project_options project_warnings)
  if(CHECK_COVERAGE)
    target_compile_options(algos PUBLIC -O0 -g --coverage)
    target_link_options(algos  PUBLIC --coverage)
  else(CHECK_COVERAGE)
    target_compile_options (algos PRIVATE -O3)
  endif(CHECK_COVERAGE)

  if (RUN_TESTS)
    add_definitions(-DRUN_TESTS)
    file(GLOB LIB_TESTS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
    enable_testing()
    add_subdirectory(tests)
  endif(RUN_TESTS)

endif (BUILD_INDIVIDUAL)
