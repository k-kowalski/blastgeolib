cmake_minimum_required(VERSION 3.10)
project(blastgeolib)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

set(FETCHCONTENT_BASE_DIR ${PROJECT_SOURCE_DIR}/deps)

if(NOT TARGET Eigen3::Eigen)
    include(FetchContent)

    set(FETCHCONTENT_BASE_DIR ${PROJECT_SOURCE_DIR}/deps)

    FetchContent_Declare(
      eigen
      GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
      GIT_TAG 3.4.0
      GIT_SHALLOW TRUE
    )

    FetchContent_MakeAvailable(eigen)
endif()


add_library(blastgeolib
    src/PolygonMesh.cpp
    src/MeshSerializer.cpp
    src/OBJSerializer.cpp
    src/STLSerializer.cpp
    src/Utils.cpp
)

target_include_directories(blastgeolib
    PRIVATE ${PROJECT_SOURCE_DIR}/include/blastgeolib
)

target_include_directories(blastgeolib
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(blastgeolib PUBLIC Eigen3::Eigen)


# testing
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
enable_testing()

add_executable(
  test_main
  tests/test_main.cpp
)
target_link_libraries(
  test_main
  blastgeolib
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(test_main)


