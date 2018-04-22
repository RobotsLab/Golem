###############################################################################
#
# Golem Plugin/Ctrl/SixAxisSim library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_CTRL)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_CTRL}/SixAxisSim")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemCtrlSixAxisSim SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if (WIN32)
		TARGET_COMPILE_OPTIONS(GolemCtrlSixAxisSim PUBLIC ${CMAKE_DLL_EXPORT_FLAGS} /wd4091)
	elseif (UNIX)
		#TARGET_COMPILE_OPTIONS(GolemCtrlSixAxisSim PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	endif ()

	TARGET_LINK_LIBRARIES(GolemCtrlSixAxisSim GolemCtrlSingleCtrl)
	
	TARGET_INCLUDE_DIRECTORIES(GolemCtrlSixAxisSim PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemCtrlSixAxisSim ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemCtrlSixAxisSim RUNTIME DESTINATION bin COMPONENT ctrl_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemCtrlSixAxisSim LIBRARY DESTINATION bin COMPONENT ctrl_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Ctrl/SixAxisSim/ COMPONENT ctrl_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT ctrl_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemCtrlSixAxisSim PROPERTY PROJECT_LABEL "SixAxisSim")
	SET_PROPERTY(TARGET GolemCtrlSixAxisSim PROPERTY FOLDER "Plugin/Ctrl")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_CTRL)
