project(utils)

cmake_minimum_required(VERSION 3.5)

add_library(utils)
target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_sources(utils PRIVATE src/utils.cpp)
