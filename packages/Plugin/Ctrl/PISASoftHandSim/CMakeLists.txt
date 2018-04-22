###############################################################################
#
# Golem Plugin/Ctrl/PISASoftHandSim library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_CTRL)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_CTRL}/PISASoftHandSim")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemCtrlPISASoftHandSim SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if (WIN32)		
		TARGET_COMPILE_OPTIONS(GolemCtrlPISASoftHandSim PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif (UNIX)
	endif()	

	TARGET_LINK_LIBRARIES(GolemCtrlPISASoftHandSim GolemCtrlPISA GolemCtrlSingleCtrl)
	
	TARGET_INCLUDE_DIRECTORIES(GolemCtrlPISASoftHandSim PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemCtrlPISASoftHandSim ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemCtrlPISASoftHandSim RUNTIME DESTINATION bin COMPONENT ctrl_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemCtrlPISASoftHandSim LIBRARY DESTINATION bin COMPONENT ctrl_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Ctrl/PISASoftHandSim/ COMPONENT ctrl_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT ctrl_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemCtrlPISASoftHandSim PROPERTY PROJECT_LABEL "PISASoftHandSim")
	SET_PROPERTY(TARGET GolemCtrlPISASoftHandSim PROPERTY FOLDER "Plugin/Ctrl")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_CTRL)
