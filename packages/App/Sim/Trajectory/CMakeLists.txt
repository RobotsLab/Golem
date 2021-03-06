###############################################################################
#
# Golem App/Sim/Trajectory application
#
###############################################################################

if(GOLEM_BUILD_APP_SIM)
	SET(PACKAGE_DIR "${PACKAGE_APP_SIM}/Trajectory")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_EXECUTABLE(GolemAppTrajectory ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	TARGET_LINK_LIBRARIES(GolemAppTrajectory GolemAppCommon)
	TARGET_INCLUDE_DIRECTORIES(GolemAppTrajectory PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemAppTrajectory ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})
	SET_PROPERTY(TARGET GolemAppTrajectory PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
	SET_PROPERTY(TARGET GolemAppTrajectory PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
	
	if (CPACK_BUILD_CORE)
		INSTALL(TARGETS GolemAppTrajectory RUNTIME DESTINATION bin COMPONENT app_execs)
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/App/Trajectory/ COMPONENT app_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT app_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemAppTrajectory PROPERTY PROJECT_LABEL "Trajectory")
	SET_PROPERTY(TARGET GolemAppTrajectory PROPERTY FOLDER "App/Sim")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_APP_SIM)
