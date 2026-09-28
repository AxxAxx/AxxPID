cmake_minimum_required(VERSION 3.14)

project(axxpid
        VERSION 1.0.0
        DESCRIPTION "Portable PID controller for C, with feed-forward"
        LANGUAGES C)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------

# Detect whether we are the top-level project without requiring CMake 3.21.
if(CMAKE_VERSION VERSION_LESS 3.21)
    if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        set(PROJECT_IS_TOP_LEVEL ON)
    else()
        set(PROJECT_IS_TOP_LEVEL OFF)
    endif()
endif()

option(AXXPID_USE_DOUBLE   "Run the controller in double precision" OFF)
option(AXXPID_BUILD_TUNE   "Build the tuning rules and relay autotuner" ON)
option(AXXPID_BUILD_TESTS  "Build the test suite" ${PROJECT_IS_TOP_LEVEL})
option(AXXPID_BUILD_EXAMPLES "Build the runnable examples" ${PROJECT_IS_TOP_LEVEL})
option(AXXPID_STRICT       "Treat warnings as errors" OFF)

# ---------------------------------------------------------------------------
# The library
# ---------------------------------------------------------------------------

set(AXXPID_SOURCES src/axxpid.c)
if(AXXPID_BUILD_TUNE)
    list(APPEND AXXPID_SOURCES src/axxpid_tune.c)
endif()

add_library(axxpid STATIC ${AXXPID_SOURCES})
add_library(axxpid::axxpid ALIAS axxpid)

target_include_directories(axxpid PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(axxpid PUBLIC c_std_99)

if(AXXPID_USE_DOUBLE)
    target_compile_definitions(axxpid PUBLIC AXXPID_USE_DOUBLE=1)
endif()

# The library builds clean at this warning level; keep it that way.
if(MSVC)
    set(AXXPID_WARNINGS /W4)
    if(AXXPID_STRICT)
        list(APPEND AXXPID_WARNINGS /WX)
    endif()
else()
    set(AXXPID_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion
                        -Wdouble-promotion -Wstrict-prototypes
                        -Wmissing-prototypes)
    if(AXXPID_STRICT)
        list(APPEND AXXPID_WARNINGS -Werror)
    endif()
endif()
target_compile_options(axxpid PRIVATE ${AXXPID_WARNINGS})

# No libm dependency: the library implements the one square root it needs.

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

if(AXXPID_BUILD_TESTS)
    enable_testing()

    set(AXXPID_TESTS test_core test_features test_closedloop)
    if(AXXPID_BUILD_TUNE)
        list(APPEND AXXPID_TESTS test_tune)
    endif()

    foreach(test_name IN LISTS AXXPID_TESTS)
        add_executable(${test_name} tests/${test_name}.c)
        target_link_libraries(${test_name} PRIVATE axxpid)
        target_include_directories(${test_name} PRIVATE tests)
        target_compile_options(${test_name} PRIVATE ${AXXPID_WARNINGS})
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
endif()

# ---------------------------------------------------------------------------
# Examples
# ---------------------------------------------------------------------------

if(AXXPID_BUILD_EXAMPLES)
    set(AXXPID_EXAMPLES 01_minimal 02_feedforward 04_soldering_iron)
    if(AXXPID_BUILD_TUNE)
        list(APPEND AXXPID_EXAMPLES 03_autotune)
    endif()

    foreach(example_name IN LISTS AXXPID_EXAMPLES)
        add_executable(${example_name} examples/${example_name}.c)
        target_link_libraries(${example_name} PRIVATE axxpid)
        target_compile_options(${example_name} PRIVATE ${AXXPID_WARNINGS})
    endforeach()
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

include(GNUInstallDirs)

install(TARGETS axxpid
        EXPORT axxpidTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/axxpid DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT axxpidTargets
        FILE axxpidConfig.cmake
        NAMESPACE axxpid::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/axxpid)
