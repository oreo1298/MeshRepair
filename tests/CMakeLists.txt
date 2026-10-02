add_executable(meshrepair-tests test_repair.cpp)
target_link_libraries(meshrepair-tests PRIVATE meshrepair)
add_test(NAME meshrepair-tests COMMAND meshrepair-tests)
