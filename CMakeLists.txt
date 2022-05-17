cmake_minimum_required(VERSION 3.14)
project(physics-engine C CXX)

set(CMAKE_CXX_STANDARD 20)

include_directories(physics-engine/include)

add_subdirectory(dependencies/google-test)
add_subdirectory(physics-engine)
add_subdirectory(physics-engine-tests)
