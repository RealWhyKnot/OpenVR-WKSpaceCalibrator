cmake_minimum_required(VERSION 3.14)

# GoogleTest pulled in via FetchContent: keeps the URL pin co-located with the
# test target (no .gitmodules churn) and makes upgrading a one-line change. The
# tag is pinned to a release rather than a moving branch so the same commit of
# this repo always builds against the same test framework version.
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# Required for MSVC: the rest of the project links against the dynamic CRT
# (/MD). Without this, gtest defaults to the static CRT (/MT) and the link
# fails with LNK2038 ('RuntimeLibrary' mismatch).
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build the test executable by compiling CalibrationCalc.cpp directly into it
# (alongside our minimal stubs). This keeps the test target decoupled from the
# overlay's GUI/OpenVR runtime dependencies — the math is pure Eigen and openvr
# header types, both of which are header-only as far as the linker is concerned.
add_executable(spacecal_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/test_calibration_calc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_protocol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_configuration.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_wedge_detector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_stall_decisions.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_geometry_shift_detector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_watchdog_decisions.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_state_transitions.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_motion_gate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_latency_estimator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_tilt_diagnostic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_robust_scale.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_blend_filter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_rest_locked_yaw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_recovery_delta_buffer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_reanchor_chi_square.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test_translation_solve_direct.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/CalibrationCalc.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/TranslationSolveDirect.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/Configuration.cpp
)

target_include_directories(spacecal_tests
    PRIVATE ${CMAKE_SOURCE_DIR}
    PRIVATE ${CMAKE_SOURCE_DIR}/src/common
    PRIVATE ${CMAKE_SOURCE_DIR}/src/overlay
    # Protocol.h and MotionGate.h moved into the OpenVR-WKPairDriver submodule;
    # tests pick them up from there so test_protocol / test_motion_gate stay
    # in lock-step with the shared wire layout.
    PRIVATE ${CMAKE_SOURCE_DIR}/lib/OpenVR-WKPairDriver/src/common
    PRIVATE ${CMAKE_SOURCE_DIR}/lib
    PRIVATE ${CMAKE_SOURCE_DIR}/lib/openvr/headers
    PRIVATE ${CMAKE_SOURCE_DIR}/lib/Eigen
)

target_link_libraries(spacecal_tests
    PRIVATE GTest::gtest_main
    PRIVATE ${CMAKE_SOURCE_DIR}/lib/openvr/lib/win64/openvr_api.lib
)

target_compile_definitions(spacecal_tests
    PRIVATE NOMINMAX
    PRIVATE UNICODE
)

# CalibrationCalc.cpp emits a large amount of Eigen template code; mirror the
# overlay's /bigobj setting so the .obj doesn't blow past the COFF section
# limit on MSVC.
if (MSVC)
    target_compile_options(spacecal_tests PRIVATE /bigobj)
endif()

# Drop the test executable next to the rest of the build artifacts so CI can
# locate it at the same predictable path the overlay uses.
adjust_bin_paths(spacecal_tests)

# Copy openvr_api.dll next to the test exe so it can launch on systems where
# the DLL isn't on PATH.
add_custom_command(
    TARGET spacecal_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${CMAKE_SOURCE_DIR}/lib/openvr/bin/win64/openvr_api.dll
    ${CMAKE_BINARY_DIR}/artifacts/$<CONFIG>/openvr_api.dll
)

include(GoogleTest)
gtest_discover_tests(spacecal_tests)

set_property(TARGET spacecal_tests PROPERTY FOLDER "tests")
# The FetchContent-imported gtest targets dump straight into the top-level
# folder without this; nudge them under "lib" so the solution stays tidy.
foreach(_gtest_target gtest gtest_main gmock gmock_main)
    if (TARGET ${_gtest_target})
        set_property(TARGET ${_gtest_target} PROPERTY FOLDER "lib/googletest")
    endif()
endforeach()
