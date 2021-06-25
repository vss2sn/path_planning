project(utils)

cmake_minimum_required(VERSION 3.15)

add_library(utils)
target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_sources(utils PRIVATE src/utils.cpp)
set_target_properties(utils
  PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wall -Werror -g"
)
target_link_libraries(utils PRIVATE project_options project_warnings)
