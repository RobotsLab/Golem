###############################################################################
#
# Golem Plugin/Data/R2GTrajectory library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_HBPLANNER)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_DATA}/R2GTrajectory")

	# OpenCV
	FIND_PACKAGE(OpenCV REQUIRED)

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemDataR2GTrajectory SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if(WIN32)
		TARGET_COMPILE_OPTIONS(GolemDataR2GTrajectory PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif(UNIX)
		TARGET_COMPILE_OPTIONS(GolemDataR2GTrajectory PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	endif()

	TARGET_LINK_LIBRARIES(GolemDataR2GTrajectory GolemHBPlanner GolemApp)
	
	TARGET_INCLUDE_DIRECTORIES(GolemDataR2GTrajectory PUBLIC ${PACKAGE_DIR}/include ${OPENCV_INCLUDE_DIRS})

	COPY_FILES(GolemDataR2GTrajectory ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemDataR2GTrajectory RUNTIME DESTINATION bin COMPONENT data_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemDataR2GTrajectory LIBRARY DESTINATION bin COMPONENT data_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Data/R2GTrajectory/ COMPONENT data_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT data_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemDataR2GTrajectory PROPERTY PROJECT_LABEL "R2GTrajectory")
	SET_PROPERTY(TARGET GolemDataR2GTrajectory PROPERTY FOLDER "Plugin/Data")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_HBPLANNER)
