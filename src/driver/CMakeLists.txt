cmake_minimum_required(VERSION 3.15)

set(SPACECAL_PAIRDRIVER_ROOT "${OPENVR_PAIRDRIVER_ROOT}" CACHE PATH "OpenVR-WKPairDriver checkout")
if(NOT SPACECAL_PAIRDRIVER_ROOT)
	set(_sibling_pairdriver "${CMAKE_CURRENT_LIST_DIR}/../../../OpenVR-WKPairDriver")
	if(EXISTS "${_sibling_pairdriver}/src/driver/DriverModule.h")
		get_filename_component(SPACECAL_PAIRDRIVER_ROOT "${_sibling_pairdriver}" ABSOLUTE)
	else()
		get_filename_component(SPACECAL_PAIRDRIVER_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../lib/OpenVR-WKPairDriver" ABSOLUTE)
	endif()
endif()

add_library(openvr_pair_feature_calibration_driver STATIC
	CalibrationDriverModule.cpp)

target_include_directories(openvr_pair_feature_calibration_driver PUBLIC
	${CMAKE_CURRENT_LIST_DIR}
	${SPACECAL_PAIRDRIVER_ROOT}/src/driver
	${SPACECAL_PAIRDRIVER_ROOT}/src/common
	${SPACECAL_PAIRDRIVER_ROOT}/lib
	${SPACECAL_PAIRDRIVER_ROOT}/lib/Eigen
	${SPACECAL_PAIRDRIVER_ROOT}/lib/openvr/headers)

target_compile_definitions(openvr_pair_feature_calibration_driver PRIVATE
	NOMINMAX
	UNICODE
	OPENVRPAIRDRIVER_EXPORTS)
