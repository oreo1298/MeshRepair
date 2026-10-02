add_library(meshrepair STATIC
    src/Mesh.cpp
    src/Diagnostics.cpp
    src/Internal.cpp
    src/Topology.cpp
    src/Stitch.cpp
    src/HoleFill.cpp
    src/Repair.cpp
    src/IO.cpp
    src/IO3MF.cpp
)
target_include_directories(meshrepair PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
set_target_properties(meshrepair PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(meshrepair PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()
