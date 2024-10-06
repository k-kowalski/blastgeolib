cmake_minimum_required(VERSION 3.10)
project(BlastGeoApp)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(.. ${CMAKE_BINARY_DIR}/BlastGeoApp)

add_executable(BlastGeoApp BlastGeoApp.cpp)

target_link_libraries(BlastGeoApp blastgeolib)
