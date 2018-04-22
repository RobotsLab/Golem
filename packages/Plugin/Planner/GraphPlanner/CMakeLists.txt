###############################################################################
#
# Golem Plugin/Planner/GraphPlanner library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_PLANNER)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_PLANNER}/GraphPlanner")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemPlannerGraphPlanner SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
		
	TARGET_COMPILE_OPTIONS(GolemPlannerGraphPlanner PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})

	TARGET_LINK_LIBRARIES(GolemPlannerGraphPlanner GolemPlanner)
	
	TARGET_INCLUDE_DIRECTORIES(GolemPlannerGraphPlanner PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemPlannerGraphPlanner ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemPlannerGraphPlanner RUNTIME DESTINATION bin COMPONENT planner_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemPlannerGraphPlanner LIBRARY DESTINATION bin COMPONENT planner_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Planner/GraphPlanner/ COMPONENT planner_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT planner_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemPlannerGraphPlanner PROPERTY PROJECT_LABEL "GraphPlanner")
	SET_PROPERTY(TARGET GolemPlannerGraphPlanner PROPERTY FOLDER "Plugin/Planner")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_PLANNER)
