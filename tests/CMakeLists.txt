add_executable(meshrepair-tests test_repair.cpp)
target_link_libraries(meshrepair-tests PRIVATE meshrepair)
add_test(NAME meshrepair-tests COMMAND meshrepair-tests)

# Low level ZIP / DEFLATE tests (internal API).
add_executable(meshrepair-zip-tests test_zip.cpp ${PROJECT_SOURCE_DIR}/libmeshrepair/src/Zip.cpp)
target_include_directories(meshrepair-zip-tests PRIVATE ${PROJECT_SOURCE_DIR}/libmeshrepair/src)
add_test(NAME meshrepair-zip-tests COMMAND meshrepair-zip-tests)
