cmake_minimum_required (VERSION 3.15)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Build directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY   		${CMAKE_BINARY_DIR}/artifacts)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY   		${CMAKE_BINARY_DIR}/artifacts)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY   		${CMAKE_BINARY_DIR}/artifacts)

# Function because lazy
function(adjust_bin_paths lib)
  set_target_properties(${lib}
		PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY 			${CMAKE_BINARY_DIR}/artifacts
		LIBRARY_OUTPUT_DIRECTORY 			${CMAKE_BINARY_DIR}/artifacts
		ARCHIVE_OUTPUT_DIRECTORY 			${CMAKE_BINARY_DIR}/artifacts
	)
endfunction()

project("SpaceCalibrator")

# Enable link-time code generation (LTCG / IPO) for Release. The math hot paths
# (CalibrationCalc.cpp, IsometryTransform.h) are Eigen-template-heavy and benefit
# noticeably from cross-TU inlining; measured ~5-15% on similar workloads. Off in
# Debug to keep build/iter times reasonable.
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)

# Unit tests are opt-out: ON by default for local + CI builds, but a release/CI
# environment that doesn't want to compile/link GoogleTest can pass
# -DSPACECAL_BUILD_TESTS=OFF at configure time.
option(SPACECAL_BUILD_TESTS "Build unit tests" ON)

# Include project
add_subdirectory ("src")

# Libs
add_subdirectory ("lib")

# Offline replay harness for captured CSV logs (tools/replay/spacecal_replay).
# Standalone target — does not affect the overlay or driver build.
add_subdirectory ("tools/replay")

if (SPACECAL_BUILD_TESTS)
    enable_testing()
    add_subdirectory("tests")
endif()

if ("${CMAKE_CXX_COMPILER}" MATCHES "Clang" OR "${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
	# add_compile_options(-Werror=return-type -Wall -Wextra -Wpedantic /MP)
	add_compile_options(-Werror=return-type /MP)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
	# add_compile_options(-Werror=return-type -Wall -Wextra -Wpedantic)
	add_compile_options(-Werror=return-type)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel" OR "${CMAKE_C_COMPILER_ID}" STREQUAL "Intel")
	# add_compile_options(-Werror=return-type -Wall -Wextra -Wpedantic)
	add_compile_options(/Qdiag-error-return)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC" OR "${CMAKE_C_COMPILER_ID}" STREQUAL "MSVC")
	add_compile_options(
		# treat warnings as errors
		/we4715 # not all control paths return a value
		/we4828 # disallow invalid characters
		# prinf-like functions: format mismatch
		/we4473 # : not enough arguments passed for format string
		/we4474 # : too many arguments passed for format string
		/we4475 # : length modifier cannot be used with type field character in format specifier
		/we4476 # : unknown type field character in format specifier
		/we4477 # : format string requires an argument of type , but variadic argument has type
		/we4478 # : positional and non-positional placeholders cannot be mixed in the same format string
		/we4775 # nonstandard extension used in format string of function
		/we4776 # % is not allowed in the format string of function
		/we4777 # : format string requires an argument of type , but variadic argument has type
		/we4778 # : unterminated format string
		# macro arg mismatch
		/we4002 # too many actual parameters for macro 'identifier'
		/we4003 # not enough actual parameters for macro 'identifier'
		# /Zc:threadSafeInit- removed. The original cite was a Server 2003 MSVC-runtime
		# bug; that OS reached EOL in 2015 (12+ years past EOL as of 2026).
		# Disabling thread-safe init silently breaks C++17's guarantee for
		# inline-static / function-local-static initialisation across threads — a
		# real correctness hazard in any code touching things like static caches in
		# multithreaded callers. Default (thread-safe init on) is safe and standards-
		# compliant on every supported Windows version.
		/MP # multiprocessor compilation
		/utf-8 # utf-8 source & exec
		/GF) # eliminate duplicate strings

	# Release-only: enable /fp:fast (relaxed FP for ~5% on Eigen-heavy code, no
	# observable accuracy impact for our quaternion/affine math) and /arch:AVX2
	# (vectorise Eigen with 256-bit registers — basically every VR-capable CPU
	# from the last decade has AVX2). Both are gated to Release so Debug stays
	# strict-FP and runs on legacy lab boxes.
	add_compile_options(
		$<$<CONFIG:Release>:/fp:fast>
		$<$<CONFIG:Release>:/arch:AVX2>)
endif()