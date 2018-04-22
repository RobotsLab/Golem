###############################################################################
#
# Golem Plugin/Ctrl/Sim library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_CTRL)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_CTRL}/Sim")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemCtrlSim SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if (WIN32)		
		TARGET_COMPILE_OPTIONS(GolemCtrlSim PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif (UNIX)
	endif()	

	TARGET_LINK_LIBRARIES(GolemCtrlSim GolemCtrlSingleCtrl)
	
	TARGET_INCLUDE_DIRECTORIES(GolemCtrlSim PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemCtrlSim ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemCtrlSim RUNTIME DESTINATION bin COMPONENT ctrl_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemCtrlSim LIBRARY DESTINATION bin COMPONENT ctrl_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Ctrl/Sim/ COMPONENT ctrl_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT ctrl_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemCtrlSim PROPERTY PROJECT_LABEL "Sim")
	SET_PROPERTY(TARGET GolemCtrlSim PROPERTY FOLDER "Plugin/Ctrl")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_CTRL)
