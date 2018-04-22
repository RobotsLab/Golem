###############################################################################
#
# Golem Plugin/Ctrl/RobotJustin library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_CTRL)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_CTRL}/RobotJustin")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemCtrlRobotJustin SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if(WIN32)
		TARGET_COMPILE_OPTIONS(GolemCtrlRobotJustin PUBLIC ${CMAKE_DLL_EXPORT_FLAGS} PUBLIC /wd4838 /wd4311 /wd4302)
	elseif(UNIX)
		TARGET_COMPILE_OPTIONS(GolemCtrlRobotJustin PUBLIC ${CMAKE_DLL_EXPORT_FLAGS} -Wno-write-strings)
	endif()

	TARGET_LINK_LIBRARIES(GolemCtrlRobotJustin GolemSys GolemCtrlKuka GolemCtrlDLR GolemCtrlSingleCtrl GolemCtrlMultiCtrl)
	
	TARGET_INCLUDE_DIRECTORIES(GolemCtrlRobotJustin PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemCtrlRobotJustin ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemCtrlRobotJustin RUNTIME DESTINATION bin COMPONENT ctrl_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemCtrlRobotJustin LIBRARY DESTINATION bin COMPONENT ctrl_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Ctrl/RobotJustin/ COMPONENT ctrl_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT ctrl_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemCtrlRobotJustin PROPERTY PROJECT_LABEL "RobotJustin")
	SET_PROPERTY(TARGET GolemCtrlRobotJustin PROPERTY FOLDER "Plugin/Ctrl")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_CTRL)
