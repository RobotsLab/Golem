###############################################################################
#
# Golem Plugin/Ctrl/PISASoftHand library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_CTRL)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_CTRL}/PISASoftHand")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	#FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	SET(PACKAGE_HEADERS
		${PACKAGE_DIR}/include/Golem/Ctrl/PISASoftHand/PISASoftHand.h
		${PACKAGE_DIR}/include/Golem/Ctrl/PISASoftHand/Data.h
	)
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml" PACKAGE_FILES "${PACKAGE_DIR}/resources/*.conf")

	ADD_LIBRARY(GolemCtrlPISASoftHand SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if(WIN32)
		TARGET_COMPILE_OPTIONS(GolemCtrlPISASoftHand PUBLIC ${CMAKE_DLL_EXPORT_FLAGS} PUBLIC /DWIN32 /D_WIN32)
	elseif(UNIX)
		#TARGET_COMPILE_OPTIONS(GolemCtrlPISASoftHand PUBLIC ${CMAKE_DLL_EXPORT_FLAGS} PUBLIC -DLINUX -Wno-unused-result)
		TARGET_COMPILE_OPTIONS(GolemCtrlPISASoftHand PUBLIC -DLINUX -Wno-unused-result)
	endif()

	TARGET_LINK_LIBRARIES(GolemCtrlPISASoftHand GolemCtrlPISA GolemCtrlSingleCtrl)
	
	TARGET_INCLUDE_DIRECTORIES(GolemCtrlPISASoftHand PUBLIC ${PACKAGE_DIR}/include ${PACKAGE_DIR}/include/Golem/Ctrl/PISASoftHand/qbmoveAPI)

	COPY_FILES(GolemCtrlPISASoftHand ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemCtrlPISASoftHand RUNTIME DESTINATION bin COMPONENT ctrl_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemCtrlPISASoftHand LIBRARY DESTINATION bin COMPONENT ctrl_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Ctrl/PISASoftHand/ COMPONENT ctrl_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT ctrl_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemCtrlPISASoftHand PROPERTY PROJECT_LABEL "PISASoftHand")
	SET_PROPERTY(TARGET GolemCtrlPISASoftHand PROPERTY FOLDER "Plugin/Ctrl")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_CTRL)
