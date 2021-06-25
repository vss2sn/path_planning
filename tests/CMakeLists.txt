project(tests)

find_package(GTest)  # For GTest
if(GTest_FOUND)  # GTestInstalled
  include_directories(${GTEST_INCLUDE_DIRS})
else(GTest_FOUND)  # Using GTEST as submodule
  include(cmake/gtest.cmake)
endif(GTest_FOUND)

if(CHECK_COVERAGE)
  LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
  include(CodeCoverage)
  APPEND_COVERAGE_COMPILER_FLAGS()
endif(CHECK_COVERAGE)

add_definitions(-DRUN_TESTS)

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
  target_link_libraries( ${test_output_file} PRIVATE ${GTEST_LIBRARIES} pthread utils algos gtest)
  add_test(${test_output_file} ${test_output_file})
endfunction()

# add_test_file(${CMAKE_CURRENT_SOURCE_DIR}/src/empty_grid_test.cpp )
# add_test_file(${CMAKE_CURRENT_SOURCE_DIR}/src/no_path_exists_test.cpp )
# add_test_file(${CMAKE_CURRENT_SOURCE_DIR}/src/path_exists_1_test.cpp )
# add_test_file(${CMAKE_CURRENT_SOURCE_DIR}/src/path_exists_2_test.cpp )
# add_test_file(${CMAKE_CURRENT_SOURCE_DIR}/src/randomly_generated_grid_test.cpp )

add_test_file(${CMAKE_CURRENT_SOURCE_DIR}/src/parameterized_tests.cpp )
