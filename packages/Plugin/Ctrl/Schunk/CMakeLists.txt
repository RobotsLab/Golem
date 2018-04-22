###############################################################################
#
# Golem Plugin/Ctrl/Schunk library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_CTRL)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_CTRL}/Schunk")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	if(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemCtrlSchunk SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
		if (WIN32)		
			TARGET_COMPILE_OPTIONS(GolemCtrlSchunk PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
		elseif (UNIX)
		endif()
	else(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemCtrlSchunk STATIC ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	endif(GOLEM_DYNAMIC_LIBS)

	TARGET_LINK_LIBRARIES(GolemCtrlSchunk GolemCtrl GolemCtrlSingleCtrl)
	
	TARGET_INCLUDE_DIRECTORIES(GolemCtrlSchunk PUBLIC ${PACKAGE_DIR}/include)

	if (CPACK_BUILD_CORE)
		if(GOLEM_DYNAMIC_LIBS)
			if (WIN32)
				INSTALL(TARGETS GolemCtrlSchunk RUNTIME DESTINATION bin COMPONENT ctrl_libs)
			elseif (UNIX)
				INSTALL(TARGETS GolemCtrlSchunk LIBRARY DESTINATION bin COMPONENT ctrl_libs)
			endif()
		else(GOLEM_DYNAMIC_LIBS)
			INSTALL(TARGETS GolemCtrlSchunk ARCHIVE DESTINATION lib COMPONENT ctrl_libs)
		endif(GOLEM_DYNAMIC_LIBS)
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Ctrl/Schunk/ COMPONENT ctrl_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT ctrl_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemCtrlSchunk PROPERTY PROJECT_LABEL "Schunk")
	SET_PROPERTY(TARGET GolemCtrlSchunk PROPERTY FOLDER "Plugin/Ctrl")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_CTRL)
