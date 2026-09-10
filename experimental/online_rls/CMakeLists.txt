cmake_minimum_required(VERSION 3.16)

# This directory can be built on its own or enabled by the parent project.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(shadow_rls_experiment LANGUAGES C)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    set(CMAKE_C_EXTENSIONS OFF)
    option(YAW_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
    include(CTest)
endif()

add_library(shadow_rls STATIC src/rls_shadow.c)

# Bind the host library to the source/header actually selected at configure
# time. CMake reruns configuration when either input changes. The Python runner
# refuses an old or foreign --library before passing a ctypes state to C.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/rls_shadow.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/rls_shadow.h")
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/src/rls_shadow.c" RLS_SOURCE_SHA256)
file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/include/rls_shadow.h" RLS_HEADER_SHA256)
set(RLS_COMPILER "${CMAKE_C_COMPILER_ID} ${CMAKE_C_COMPILER_VERSION} (CMake)")
configure_file(src/rls_provenance.c.in rls_provenance.c @ONLY)
add_library(shadow_rls_host SHARED src/rls_shadow.c
    "${CMAKE_CURRENT_BINARY_DIR}/rls_provenance.c")

foreach(rls_target IN ITEMS shadow_rls shadow_rls_host)
    target_include_directories(${rls_target} PUBLIC include)
    target_link_libraries(${rls_target} PUBLIC m)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${rls_target} PRIVATE -Wall -Wextra -Wpedantic
            -Werror -Wconversion -Wshadow)
        if(YAW_SANITIZERS)
            target_compile_options(${rls_target} PUBLIC
                -fsanitize=address,undefined -fno-omit-frame-pointer)
            target_link_options(${rls_target} PUBLIC -fsanitize=address,undefined)
        endif()
    endif()
endforeach()

if(BUILD_TESTING)
    add_executable(test_rls_shadow tests/test_rls_shadow.c)
    target_link_libraries(test_rls_shadow PRIVATE shadow_rls)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(test_rls_shadow PRIVATE -Wall -Wextra -Wpedantic
            -Werror -Wconversion -Wshadow -UNDEBUG)
    endif()
    add_test(NAME rls_shadow COMMAND test_rls_shadow)
endif()
