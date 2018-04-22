###############################################################################
#
# Golem Interop/Loader application
#
###############################################################################

option(GOLEM_BUILD_INTEROP_LOADER "Build Golem Interop Loader" YES)
mark_as_advanced(GOLEM_BUILD_INTEROP_LOADER)

if(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_LOADER)
	SET(PACKAGE_DIR "${PACKAGE_INTEROP}/Loader")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.config")

	ADD_EXECUTABLE(GolemInteropLoader ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	TARGET_LINK_LIBRARIES(GolemInteropLoader GolemInteropCore)
	TARGET_INCLUDE_DIRECTORIES(GolemInteropLoader PUBLIC ${PACKAGE_DIR}/include PUBLIC ${GOLEM_PROJECT_PACKAGES}/Interop/Loader/include/Golem/Interop/Loader)

	COPY_FILES(GolemInteropLoader ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})
	SET_PROPERTY(TARGET GolemInteropLoader PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
	SET_PROPERTY(TARGET GolemInteropLoader PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
	
	if (CPACK_BUILD_CORE)
		INSTALL(TARGETS GolemInteropLoader RUNTIME DESTINATION bin COMPONENT core_execs)
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Interop/Loader/ COMPONENT core_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT core_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemInteropLoader PROPERTY PROJECT_LABEL "Loader")
	SET_PROPERTY(TARGET GolemInteropLoader PROPERTY FOLDER "Interop")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_LOADER)

###############################################################################