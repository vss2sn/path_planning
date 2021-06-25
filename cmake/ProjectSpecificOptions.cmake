# If building each file with it's own output, set to ON
# If buiding single file (main.cpp) to test any/all protocols, set to OFF

# allow dependent options in CMake
include(CMakeDependentOption)

option( BUILD_INDIVIDUAL "Build each algorithm .cpp file with its own executable" OFF)
# default OFF
cmake_dependent_option( RUN_TESTS "Build and run tests" OFF "NOT BUILD_INDIVIDUAL" OFF)
# default OFF, change by user input if and only if condition allows (BUILD_INDIVIDUAL=OFF)
cmake_dependent_option( CHECK_COVERAGE "Run code coverage check" OFF "RUN_TESTS" OFF)
# default OFF, change by user input if and only if condition allows (RUN_TESTS=ON)
cmake_dependent_option( CUSTOM_DEBUG_HELPER_FUNCION "Build custom debug helper functions" ON "NOT ENABLE_COVERAGE" OFF)
# default ON, change by user input if and only if condition allows (ENABLE_COVERAGE=OFF)

if(CUSTOM_DEBUG_HELPER_FUNCION)
  add_definitions(-DCUSTOM_DEBUG_HELPER_FUNCION)
endif(CUSTOM_DEBUG_HELPER_FUNCION)
