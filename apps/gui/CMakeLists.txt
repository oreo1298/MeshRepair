# meshrepair-gui: Dear ImGui + GLFW + OpenGL 3.3 viewer.

option(MESHREPAIR_FETCH_GLFW "Download and build GLFW if it is not installed" ON)

find_package(Threads REQUIRED)

find_package(glfw3 3.3 QUIET)
if(TARGET glfw)
    set(MESHREPAIR_GLFW_TARGET glfw)
else()
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(GLFW3 QUIET IMPORTED_TARGET glfw3)
    endif()
    if(TARGET PkgConfig::GLFW3)
        set(MESHREPAIR_GLFW_TARGET PkgConfig::GLFW3)
    elseif(MESHREPAIR_FETCH_GLFW)
        message(STATUS "GLFW not found, building it from source")
        include(FetchContent)
        set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(glfw
            URL https://github.com/glfw/glfw/releases/download/3.4/glfw-3.4.zip
            URL_HASH SHA256=b5ec004b2712fd08e8861dc271428f048775200a2df719ccf575143ba749a3e9)
        FetchContent_MakeAvailable(glfw)
        set(MESHREPAIR_GLFW_TARGET glfw)
    else()
        message(FATAL_ERROR "GLFW 3.3+ not found. Install it (Arch: glfw, Debian/Ubuntu: libglfw3-dev, "
                            "Fedora: glfw-devel) or configure with -DMESHREPAIR_BUILD_GUI=OFF")
    endif()
endif()

set(IMGUI_DIR ${PROJECT_SOURCE_DIR}/third_party/imgui)
add_library(meshrepair_imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(meshrepair_imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)
target_compile_definitions(meshrepair_imgui PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)
target_link_libraries(meshrepair_imgui PUBLIC ${MESHREPAIR_GLFW_TARGET} ${CMAKE_DL_LIBS})

add_executable(meshrepair-gui
    main.cpp
    GL.cpp
    Viewer.cpp
    FileDialog.cpp
    Platform.cpp
    Png.cpp
)
target_compile_definitions(meshrepair-gui PRIVATE MESHREPAIR_VERSION="${PROJECT_VERSION}")
target_link_libraries(meshrepair-gui PRIVATE meshrepair meshrepair_imgui Threads::Threads)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(meshrepair-gui PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

install(TARGETS meshrepair-gui RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
