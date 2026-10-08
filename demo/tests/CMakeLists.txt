# Lua tests for the generated Bullet3 bindings.
#
#   cmake --build <build> && ctest --test-dir <build> --output-on-failure
#
# Each test_*.lua file is one CTest test, run by bullet3_test in a fresh Lua state with the
# bindings registered. test_api_coverage.lua checks every bound class and function against
# bullet3_api.lua, a manifest regenerated from bullet3_bindings.cpp on every build.

set(API_MANIFEST "${CMAKE_CURRENT_BINARY_DIR}/bullet3_api.lua")

# Regenerated on every build: it takes a moment, and a generated file (bullet3_bindings.cpp) can
# only be named as a dependency in the directory that defines its rule, which is not this one.
# Depending on the bindings library makes sure the file exists by the time the script reads it.
add_custom_target(bullet3_api_manifest
    COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/gen_api_manifest.py"
        "${GENERATED_BINDINGS_FILE}"
        "${API_MANIFEST}"
    BYPRODUCTS "${API_MANIFEST}"
    COMMENT "Listing the bound Bullet3 classes and functions for the tests"
    VERBATIM
)
add_dependencies(bullet3_api_manifest bullet3_lua_bindings)

add_executable(bullet3_test test_main.cpp)

target_compile_definitions(bullet3_test PRIVATE
    BULLET3_TEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    BULLET3_TEST_GENERATED_DIR="${CMAKE_CURRENT_BINARY_DIR}"
)

target_link_libraries(bullet3_test PRIVATE bullet3_lua_bindings)
add_dependencies(bullet3_test bullet3_api_manifest)

if(WIN32)
    target_compile_definitions(bullet3_test PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# Re-run the configure step when a test file is added or removed
file(GLOB BULLET3_LUA_TESTS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test_*.lua")

foreach(test_file IN LISTS BULLET3_LUA_TESTS)
    get_filename_component(test_name "${test_file}" NAME_WE)
    add_test(NAME ${test_name} COMMAND bullet3_test "${test_file}")
    set_tests_properties(${test_name} PROPERTIES TIMEOUT 300)
endforeach()
