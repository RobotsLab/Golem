###############################################################################
#
# Golem Core/Planner library
#
###############################################################################

if(GOLEM_BUILD_CORE)
	SET(PACKAGE_DIR "${PACKAGE_CORE}/Planner")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")

	if(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemPlanner SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS})		
		TARGET_COMPILE_OPTIONS(GolemPlanner PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	else(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemPlanner STATIC ${PACKAGE_SOURCES} ${PACKAGE_HEADERS})
	endif(GOLEM_DYNAMIC_LIBS)

	TARGET_LINK_LIBRARIES(GolemPlanner GolemCtrl)
	
	TARGET_INCLUDE_DIRECTORIES(GolemPlanner PUBLIC ${PACKAGE_DIR}/include)
	
	if (CPACK_BUILD_CORE)
		if(GOLEM_DYNAMIC_LIBS)
			if (WIN32)
				INSTALL(TARGETS GolemPlanner RUNTIME DESTINATION bin COMPONENT core_libs)
			elseif (UNIX)
				INSTALL(TARGETS GolemPlanner LIBRARY DESTINATION bin COMPONENT core_libs)
			endif()
		else(GOLEM_DYNAMIC_LIBS)
			INSTALL(TARGETS GolemPlanner ARCHIVE DESTINATION lib COMPONENT core_libs)
		endif(GOLEM_DYNAMIC_LIBS)
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Planner/ COMPONENT core_headers)
	endif (CPACK_BUILD_CORE)
	
	SET_PROPERTY(TARGET GolemPlanner PROPERTY PROJECT_LABEL "Planner")
	SET_PROPERTY(TARGET GolemPlanner PROPERTY FOLDER "Core")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
endif(GOLEM_BUILD_CORE)
