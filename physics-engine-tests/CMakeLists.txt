set(BINARY ${CMAKE_PROJECT_NAME}-tst)

add_executable(${BINARY} src/main.cpp)

add_test(NAME ${BINARY} COMMAND ${BINARY})

target_link_libraries(${BINARY} PUBLIC ${CMAKE_PROJECT_NAME}-lib gtest)
