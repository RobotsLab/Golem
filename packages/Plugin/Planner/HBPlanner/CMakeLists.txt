###############################################################################
#
# Golem Plugin/Planner/HBPlanner library
#
###############################################################################

option(GOLEM_BUILD_PLUGIN_HBPLANNER "Build Golem Plugin/HBPlanner plugins" NO)
mark_as_advanced(GOLEM_BUILD_PLUGIN_HBPLANNER)

if(GOLEM_BUILD_PLUGIN_HBPLANNER)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_PLANNER}/HBPlanner")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	#FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemHBPlanner SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	TARGET_COMPILE_OPTIONS(GolemHBPlanner PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})

	TARGET_LINK_LIBRARIES(GolemHBPlanner GolemPlannerGraphPlanner GolemTools GolemSys GolemContact)
	
	TARGET_INCLUDE_DIRECTORIES(GolemHBPlanner PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemHBPlanner ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemHBPlanner RUNTIME DESTINATION bin COMPONENT planner_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemHBPlanner LIBRARY DESTINATION bin COMPONENT planner_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/HBPlanner/ COMPONENT planner_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT planner_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemHBPlanner PROPERTY PROJECT_LABEL "HBPlanner")
	SET_PROPERTY(TARGET GolemHBPlanner PROPERTY FOLDER "Plugin/Planner")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_HBPLANNER)
