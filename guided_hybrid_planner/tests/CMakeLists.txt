# set test build target name

set(TEST_BINARY ${PROJECT_NAME}_test)

add_executable(${TEST_BINARY} test.cpp)

add_subdirectory(googletest)

include_directories(googletest/include)

target_link_libraries(${TEST_BINARY} PUBLIC gtest ${library_name})

install(TARGETS ${TEST_BINARY}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# target_link_libraries(${TEST_BINARY} )