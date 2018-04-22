###############################################################################
#
# Golem Plugin/Ctrl/SingleCtrl library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_CTRL)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_CTRL}/SingleCtrl")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	if(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemCtrlSingleCtrl SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
		if (WIN32)		
			TARGET_COMPILE_OPTIONS(GolemCtrlSingleCtrl PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
		elseif (UNIX)
		endif()
	else(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemCtrlSingleCtrl STATIC ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	endif(GOLEM_DYNAMIC_LIBS)

	TARGET_LINK_LIBRARIES(GolemCtrlSingleCtrl GolemCtrl)
	
	TARGET_INCLUDE_DIRECTORIES(GolemCtrlSingleCtrl PUBLIC ${PACKAGE_DIR}/include)

	if (CPACK_BUILD_CORE)
		if(GOLEM_DYNAMIC_LIBS)
			if (WIN32)
				INSTALL(TARGETS GolemCtrlSingleCtrl RUNTIME DESTINATION bin COMPONENT ctrl_libs)
			elseif (UNIX)
				INSTALL(TARGETS GolemCtrlSingleCtrl LIBRARY DESTINATION bin COMPONENT ctrl_libs)
			endif()
		else(GOLEM_DYNAMIC_LIBS)
			INSTALL(TARGETS GolemCtrlSingleCtrl ARCHIVE DESTINATION lib COMPONENT ctrl_libs)
		endif(GOLEM_DYNAMIC_LIBS)
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Ctrl/SingleCtrl/ COMPONENT ctrl_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT ctrl_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemCtrlSingleCtrl PROPERTY PROJECT_LABEL "SingleCtrl")
	SET_PROPERTY(TARGET GolemCtrlSingleCtrl PROPERTY FOLDER "Plugin/Ctrl")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_CTRL)
