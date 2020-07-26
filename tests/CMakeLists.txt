project(tests)

if(CHECK_COVERAGE)
  LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
  include(CodeCoverage)
  APPEND_COVERAGE_COMPILER_FLAGS()
endif(CHECK_COVERAGE)

# For GTEST
find_package(GTest)
if(GTest_FOUND)
  # Installed
  include_directories(${GTEST_INCLUDE_DIRS})
else(GTest_FOUND)
  # Using GTEST as submodule
  include(gtest.cmake)
endif(GTest_FOUND)

add_definitions(-DRUN_TESTS)

function(add_test_file TEST_SOURCE)
  get_filename_component(test_output_file ${TEST_SOURCE} NAME_WE)
  add_executable( ${test_output_file} )
  set_target_properties(${test_output_file} PROPERTIES CXX_STANDARD 17)
  target_sources(${test_output_file} PRIVATE ${TEST_SOURCE})
  target_include_directories(${test_output_file} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries( ${test_output_file} ${GTEST_LIBRARIES} pthread utils algos gtest)
  if(CHECK_COVERAGE)
    target_link_libraries(${test_output_file} gcov)
    SETUP_TARGET_FOR_COVERAGE_LCOV(NAME ${test_output_file}_coverage EXECUTABLE ${test_output_file} )
    add_test(${test_output_file}_coverage ${test_output_file})
    add_custom_command( TARGET ${test_output_file} POST_BUILD COMMAND make ${test_output_file}_coverage)
  # run after build else need to manaully run each one to get coverage
  else(CHECK_COVERAGE)
    add_test(${test_output_file} ${test_output_file})
  endif(CHECK_COVERAGE)
endfunction()

add_test_file(${CMAKE_CURRENT_SOURCE_DIR}/src/grid1.cpp )
add_test_file(${CMAKE_CURRENT_SOURCE_DIR}/src/grid2.cpp )
add_test_file(${CMAKE_CURRENT_SOURCE_DIR}/src/grid3.cpp )
add_test_file(${CMAKE_CURRENT_SOURCE_DIR}/src/grid4.cpp )
add_test_file(${CMAKE_CURRENT_SOURCE_DIR}/src/grid5.cpp )
add_test_file(${CMAKE_CURRENT_SOURCE_DIR}/src/gridr.cpp )
