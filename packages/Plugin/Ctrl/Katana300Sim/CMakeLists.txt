###############################################################################
#
# Golem Plugin/Ctrl/Katana300Sim library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_CTRL)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_CTRL}/Katana300Sim")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemCtrlKatana300Sim SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if (WIN32)		
		TARGET_COMPILE_OPTIONS(GolemCtrlKatana300Sim PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif (UNIX)
	endif()
	
	TARGET_LINK_LIBRARIES(GolemCtrlKatana300Sim GolemCtrlKatana GolemCtrlSingleCtrl)
	
	TARGET_INCLUDE_DIRECTORIES(GolemCtrlKatana300Sim PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemCtrlKatana300Sim ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemCtrlKatana300Sim RUNTIME DESTINATION bin COMPONENT ctrl_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemCtrlKatana300Sim LIBRARY DESTINATION bin COMPONENT ctrl_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Ctrl/Katana300Sim/ COMPONENT ctrl_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT ctrl_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemCtrlKatana300Sim PROPERTY PROJECT_LABEL "Katana300Sim")
	SET_PROPERTY(TARGET GolemCtrlKatana300Sim PROPERTY FOLDER "Plugin/Ctrl")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_CTRL)
