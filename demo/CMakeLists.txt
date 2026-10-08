cmake_minimum_required(VERSION 3.28)
project(bullet3_luabridge_demo C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Fetch Bullet3 (only the core libraries)
# Bullet3 declares an old cmake_minimum_required, make its option() calls honor the variables below
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(BUILD_BULLET3 OFF)
set(BUILD_EXTRAS OFF)
set(BUILD_UNIT_TESTS OFF)
set(BUILD_CPU_DEMOS OFF)
set(BUILD_BULLET2_DEMOS OFF)
set(BUILD_OPENGL3_DEMOS OFF)
set(BUILD_ENET OFF)
set(BUILD_CLSOCKET OFF)
set(USE_GRAPHICAL_BENCHMARK OFF)
set(INSTALL_LIBS OFF)
FetchContent_Declare(
    bullet3
    GIT_REPOSITORY https://github.com/bulletphysics/bullet3.git
    GIT_TAG master
    GIT_SHALLOW TRUE
    EXCLUDE_FROM_ALL
)

# Fetch LuaBridge3
FetchContent_Declare(
    luabridge3
    GIT_REPOSITORY https://github.com/kunitoki/LuaBridge3.git
    GIT_TAG master
    GIT_SHALLOW TRUE
)

# Fetch Lua
FetchContent_Declare(
    lua
    GIT_REPOSITORY https://github.com/lua/lua.git
    GIT_TAG v5.4.8
    GIT_SHALLOW TRUE
)

# Bullet's CMakeLists.txt is old and not ours to change, and configuring it causes two warnings:
#  - cmake_minimum_required(VERSION 3.5) is deprecated, which is silenced only while the
#    dependencies are being added, so deprecations in this project still show;
#  - FindOpenGL (CMP0072) wants to know which OpenGL libraries to prefer. Nothing here links
#    OpenGL, so take the modern default; it falls back to the legacy libraries without GLVND.
set(OpenGL_GL_PREFERENCE GLVND)

# (CMake only reads the cache entry of CMAKE_WARN_DEPRECATED, so set it there and put it back)
get_property(bullet3_warn_deprecated_set CACHE CMAKE_WARN_DEPRECATED PROPERTY VALUE SET)
get_property(bullet3_warn_deprecated CACHE CMAKE_WARN_DEPRECATED PROPERTY VALUE)
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(bullet3 luabridge3 lua)
if(bullet3_warn_deprecated_set)
    set(CMAKE_WARN_DEPRECATED ${bullet3_warn_deprecated} CACHE BOOL "" FORCE)
else()
    unset(CMAKE_WARN_DEPRECATED CACHE)
endif()

# Build Lua as a library
add_library(lua_lib STATIC ${lua_SOURCE_DIR}/onelua.c)
target_include_directories(lua_lib PUBLIC ${lua_SOURCE_DIR})
target_compile_definitions(lua_lib PRIVATE MAKE_LIB=1)
if(UNIX AND NOT APPLE)
    target_link_libraries(lua_lib PUBLIC m)
endif()

# Install upstream tsujikiri in a virtual environment
find_package(Python3 3.10 COMPONENTS Interpreter REQUIRED)

set(VENV_DIR "${CMAKE_CURRENT_BINARY_DIR}/venv")
set(VENV_STAMP "${VENV_DIR}/requirements.stamp")
set(REQUIREMENTS_FILE "${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt")

if(WIN32 AND NOT MINGW)
    set(VENV_BIN_DIR "${VENV_DIR}/Scripts")
else()
    set(VENV_BIN_DIR "${VENV_DIR}/bin")
endif()
set(VENV_PYTHON "${VENV_BIN_DIR}/python${CMAKE_EXECUTABLE_SUFFIX}")

add_custom_command(
    OUTPUT "${VENV_STAMP}"
    COMMAND ${Python3_EXECUTABLE} -m venv "${VENV_DIR}"
    COMMAND ${VENV_PYTHON} -m pip install --upgrade pip
    COMMAND ${VENV_PYTHON} -m pip install -r "${REQUIREMENTS_FILE}"
    COMMAND ${CMAKE_COMMAND} -E touch "${VENV_STAMP}"
    DEPENDS "${REQUIREMENTS_FILE}"
    COMMENT "Creating Python virtual environment and installing tsujikiri"
    VERBATIM
)

# Generate bindings using tsujikiri
set(INPUT_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/bullet3.input.yml")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/bullet3.input.yml.in"
    "${INPUT_CONFIG}"
    @ONLY
)

set(GENERATED_BINDINGS_FILE "${CMAKE_CURRENT_BINARY_DIR}/bullet3_bindings.cpp")

# The venv bin dir goes on PATH so tsujikiri finds its bundled clang-format for `pretty: true`
add_custom_command(
    OUTPUT "${GENERATED_BINDINGS_FILE}"
    COMMAND ${CMAKE_COMMAND} -E env --modify "PATH=path_list_prepend:${VENV_BIN_DIR}"
        ${VENV_PYTHON} -m tsujikiri
            --input "${INPUT_CONFIG}"
            --target luabridge3 "${GENERATED_BINDINGS_FILE}"
            --strict
    DEPENDS
        "${VENV_STAMP}"
        "${INPUT_CONFIG}"
        "${CMAKE_CURRENT_SOURCE_DIR}/bullet3_headers.h"
    COMMENT "Generating Bullet3 bindings using tsujikiri"
    VERBATIM
)

# The generated bindings are compiled once and shared by the demo and the tests
add_library(bullet3_lua_bindings STATIC "${GENERATED_BINDINGS_FILE}")

target_include_directories(bullet3_lua_bindings PUBLIC
    ${bullet3_SOURCE_DIR}/src
    ${luabridge3_SOURCE_DIR}/Source
)

target_link_libraries(bullet3_lua_bindings PUBLIC
    BulletSoftBody
    BulletDynamics
    BulletCollision
    LinearMath
    lua_lib
)

# Create the demo executable
add_executable(bullet3_demo main.cpp)

target_compile_definitions(bullet3_demo PRIVATE
    BULLET3_DEMO_SCRIPT="${CMAKE_CURRENT_SOURCE_DIR}/bullet3_demo.lua"
)

target_link_libraries(bullet3_demo PRIVATE bullet3_lua_bindings)

# Add compile definitions for Windows
if(WIN32)
    target_compile_definitions(bullet3_lua_bindings PRIVATE _CRT_SECURE_NO_WARNINGS)
    target_compile_definitions(bullet3_demo PRIVATE _CRT_SECURE_NO_WARNINGS)
elseif(APPLE)
    target_compile_options(BulletDynamics PRIVATE -Wno-everything)
    target_compile_options(BulletCollision PRIVATE -Wno-everything)
    target_compile_options(LinearMath PRIVATE -Wno-everything)
endif()

# Lua tests for the generated bindings (run with `ctest`, or `just test`)
option(BULLET3_BUILD_TESTS "Build and register the Lua binding tests" ON)
if(BULLET3_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
