project(tests)

cmake_minimum_required(VERSION 3.15)

find_package(GTest)  # For GTest
if(GTest_FOUND)  # GTestInstalled
  include_directories(${GTEST_INCLUDE_DIRS})
else(GTest_FOUND)  # Using GTEST as submodule
  include(cmake/gtest.cmake)
endif(GTest_FOUND)

add_definitions(-DRUN_TESTS)

add_library(test_utils)
target_include_directories(test_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_sources(test_utils PRIVATE src/test_utils.cpp)
set_target_properties(test_utils
  PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wall -Werror -g"
)
target_link_libraries(test_utils PRIVATE utils algos project_options project_warnings)

function(add_test_file TEST_SOURCE)
  get_filename_component(test_output_file ${TEST_SOURCE} NAME_WE)
  add_executable( ${test_output_file} )
  set_target_properties(${test_output_file}
    PROPERTIES
      CXX_STANDARD 20
      CXX_STANDARD_REQUIRED YES
      CXX_EXTENSIONS NO
      CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wall -Werror -g"
  )
  target_sources(${test_output_file} PRIVATE ${TEST_SOURCE})
  target_include_directories(${test_output_file} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries( ${test_output_file}
    PRIVATE
      utils
      algos
      ${GTEST_LIBRARIES}
      pthread
      gtest
      test_utils
      project_options
      project_warnings
    )
  add_test(${test_output_file} ${test_output_file})
endfunction()

add_test_file(${CMAKE_CURRENT_SOURCE_DIR}/src/parameterized_tests.cpp )
