###############################################################################
#
# Golem App/Tiny/TinyIcePushing application
#
###############################################################################

if(GOLEM_BUILD_APP_TINY)
	SET(PACKAGE_DIR "${PACKAGE_APP_TINY}/TinyIcePushing")

	find_package(Ice REQUIRED Ice IceUtil)
	
	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	SET(PACKAGE_SOURCES_ICE ${GOLEM_PROJECT_PACKAGES}/Tiny/TinyIce/include/Golem/TinyIce/TinyIce.cc)
	
	ADD_EXECUTABLE(GolemAppTinyIcePushing ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES} ${PACKAGE_SOURCES_ICE})
	if (WIN32)
		TARGET_LINK_LIBRARIES(GolemAppTinyIcePushing ${Ice_LIBRARIES})
		TARGET_COMPILE_OPTIONS(GolemAppTinyIcePushing PUBLIC /wd4091)
	elseif(UNIX)
		TARGET_LINK_LIBRARIES(GolemAppTinyIcePushing ${Ice_LIBRARIES} pthread)
	endif()
	add_dependencies(GolemAppTinyIcePushing GolemTinyIce)
	TARGET_INCLUDE_DIRECTORIES(GolemAppTinyIcePushing PUBLIC ${PACKAGE_DIR}/include ${Ice_INCLUDE_DIRS} ${GOLEM_PROJECT_PACKAGES}/Tiny/TinyIce/include)

	COPY_FILES(GolemAppTinyIcePushing ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})
	SET_PROPERTY(TARGET GolemAppTinyIcePushing PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
	SET_PROPERTY(TARGET GolemAppTinyIcePushing PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
	
	if (CPACK_BUILD_CORE)
		INSTALL(TARGETS GolemAppTinyIcePushing RUNTIME DESTINATION bin COMPONENT app_execs)
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/App/TinyIcePushing/ COMPONENT app_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT app_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemAppTinyIcePushing PROPERTY PROJECT_LABEL "TinyIcePushing")
	SET_PROPERTY(TARGET GolemAppTinyIcePushing PROPERTY FOLDER "App/Tiny")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_APP_TINY)
