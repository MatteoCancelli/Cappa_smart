cmake_minimum_required(VERSION 3.14)
project(Cappa_smart_test)

set(CMAKE_CXX_STANDARD 17)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

include(FetchContent)
FetchContent_Declare(
  googletest
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
FetchContent_MakeAvailable(googletest)

add_subdirectory(test)