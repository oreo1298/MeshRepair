add_executable(meshrepair-cli main.cpp)
set_target_properties(meshrepair-cli PROPERTIES OUTPUT_NAME meshrepair)
target_link_libraries(meshrepair-cli PRIVATE meshrepair)
install(TARGETS meshrepair-cli RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
