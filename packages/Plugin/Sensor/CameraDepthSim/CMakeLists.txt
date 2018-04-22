###############################################################################
#
# Golem Plugin/Sensor/CameraDepthSim library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_SENSOR)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_SENSOR}/CameraDepthSim")

	# OpenCV
	FIND_PACKAGE(OpenCV REQUIRED)

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")
	FILE(GLOB_RECURSE PACKAGE_FILES_CAL "${PACKAGE_DIR}/resources/*.cal")

	ADD_LIBRARY(GolemCameraDepthSim SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES} ${PACKAGE_FILES_CAL})
	TARGET_COMPILE_OPTIONS(GolemCameraDepthSim PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})

	TARGET_LINK_LIBRARIES(GolemCameraDepthSim GolemTools)
	
	TARGET_INCLUDE_DIRECTORIES(GolemCameraDepthSim PUBLIC ${PACKAGE_DIR}/include ${OPENCV_INCLUDE_DIRS})

	COPY_FILES(GolemCameraDepthSim ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemCameraDepthSim RUNTIME DESTINATION bin COMPONENT sensor_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemCameraDepthSim LIBRARY DESTINATION bin COMPONENT sensor_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Sensor/CameraDepthSim/ COMPONENT sensor_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT sensor_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemCameraDepthSim PROPERTY PROJECT_LABEL "CameraDepthSim")
	SET_PROPERTY(TARGET GolemCameraDepthSim PROPERTY FOLDER "Plugin/Sensor")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES} ${PACKAGE_FILES_CAL})
endif(GOLEM_BUILD_PLUGIN_SENSOR)
