cmake_minimum_required(VERSION 3.10)
project(blastgeolib)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

set(FETCHCONTENT_BASE_DIR ${PROJECT_SOURCE_DIR}/deps)

FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0  # Specify version tag
)

FetchContent_MakeAvailable(eigen)

add_library(blastgeolib
    src/PolygonMesh.cpp
)

target_include_directories(blastgeolib
    PRIVATE ${PROJECT_SOURCE_DIR}/include/blastgeolib
)

target_include_directories(blastgeolib
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(blastgeolib PUBLIC Eigen3::Eigen)
