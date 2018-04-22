###############################################################################
#
# Golem Plugin/Ctrl/SchunkDexHandSim library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_CTRL)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_CTRL}/SchunkDexHandSim")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemCtrlSchunkDexHandSim SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if (WIN32)		
		TARGET_COMPILE_OPTIONS(GolemCtrlSchunkDexHandSim PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif (UNIX)
	endif()
	
	TARGET_LINK_LIBRARIES(GolemCtrlSchunkDexHandSim GolemCtrlSchunk GolemCtrlSingleCtrl)
	
	TARGET_INCLUDE_DIRECTORIES(GolemCtrlSchunkDexHandSim PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemCtrlSchunkDexHandSim ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemCtrlSchunkDexHandSim RUNTIME DESTINATION bin COMPONENT ctrl_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemCtrlSchunkDexHandSim LIBRARY DESTINATION bin COMPONENT ctrl_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Ctrl/SchunkDexHandSim/ COMPONENT ctrl_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT ctrl_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemCtrlSchunkDexHandSim PROPERTY PROJECT_LABEL "SchunkDexHandSim")
	SET_PROPERTY(TARGET GolemCtrlSchunkDexHandSim PROPERTY FOLDER "Plugin/Ctrl")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_CTRL)
