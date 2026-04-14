add_executable(test_logic
  test_logic.cpp
  ../Logic.cpp
)

target_include_directories(test_logic PRIVATE
  ${CMAKE_SOURCE_DIR}/test
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/include
)

target_compile_definitions(test_logic PRIVATE UNIT_TEST)

target_link_libraries(test_logic gtest_main)