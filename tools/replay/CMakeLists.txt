cmake_minimum_required(VERSION 3.15)

# Offline replay harness for spacecal_log_v2 captures. Links directly against
# CalibrationCalc.cpp so we re-run the production math against captured raw
# poses; CalibrationMetrics_stub.cpp provides no-op shims for the metrics and
# CalCtx symbols that CalibrationCalc references in the live overlay context.

project(SpaceCalibratorReplay)
message("SpaceCalibrator - Replay Harness")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(REPLAY_SOURCES
    main.cpp
    CalibrationMetrics_stub.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/CalibrationCalc.cpp
    ${CMAKE_SOURCE_DIR}/src/overlay/TranslationSolveDirect.cpp
)

add_executable(spacecal_replay ${REPLAY_SOURCES})

target_include_directories(spacecal_replay
    PRIVATE ${CMAKE_SOURCE_DIR}
    PRIVATE ${CMAKE_SOURCE_DIR}/src/common
    PRIVATE ${CMAKE_SOURCE_DIR}/src/overlay
    # Protocol.h and MotionGate.h moved into the OpenVR-WKPairDriver submodule.
    PRIVATE ${CMAKE_SOURCE_DIR}/lib/OpenVR-WKPairDriver/src/common
    PRIVATE ${CMAKE_SOURCE_DIR}/lib
    PRIVATE ${CMAKE_SOURCE_DIR}/lib/openvr/headers
    PRIVATE ${CMAKE_SOURCE_DIR}/lib/Eigen
)

target_compile_definitions(spacecal_replay
    PRIVATE NOMINMAX
)

# CalibrationCalc.cpp uses Eigen, which can produce huge object files in C++20.
if (MSVC)
    target_compile_options(spacecal_replay PRIVATE /bigobj)
else ()
    target_compile_options(spacecal_replay PRIVATE -Wa,-mbig-obj)
endif ()

# Place the binary in the same artifacts/<config>/ folder as the overlay so it's
# easy to find next to the rest of the build output.
set_target_properties(spacecal_replay PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts
)
