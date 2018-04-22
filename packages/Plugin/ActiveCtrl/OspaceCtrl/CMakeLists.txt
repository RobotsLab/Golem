###############################################################################
#
# Golem Plugin/ActiveCtrl/OspaceCtrl library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_ACTIVECTRL)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_ACTIVECTRL}/OspaceCtrl")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemOspaceCtrl SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if(WIN32)
		TARGET_COMPILE_OPTIONS(GolemOspaceCtrl PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif(UNIX)
		TARGET_COMPILE_OPTIONS(GolemOspaceCtrl PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	endif()

	TARGET_LINK_LIBRARIES(GolemOspaceCtrl GolemActiveCtrl GolemTools)
	
	TARGET_INCLUDE_DIRECTORIES(GolemOspaceCtrl PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemOspaceCtrl ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemOspaceCtrl RUNTIME DESTINATION bin COMPONENT activectrl_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemOspaceCtrl LIBRARY DESTINATION bin COMPONENT activectrl_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/ActiveCtrl/OspaceCtrl/ COMPONENT activectrl_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT activectrl_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemOspaceCtrl PROPERTY PROJECT_LABEL "OspaceCtrl")
	SET_PROPERTY(TARGET GolemOspaceCtrl PROPERTY FOLDER "Plugin/ActiveCtrl")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_ACTIVECTRL)
