###############################################################################
#
# Golem Plugin/Data/ContactModel library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_DATA)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_DATA}/ContactModel")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemDataContactModel SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if(WIN32)
		TARGET_COMPILE_OPTIONS(GolemDataContactModel PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif(UNIX)
		TARGET_COMPILE_OPTIONS(GolemDataContactModel PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	endif()

	TARGET_LINK_LIBRARIES(GolemDataContactModel GolemContact)
	
	TARGET_INCLUDE_DIRECTORIES(GolemDataContactModel PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemDataContactModel ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemDataContactModel RUNTIME DESTINATION bin COMPONENT data_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemDataContactModel LIBRARY DESTINATION bin COMPONENT data_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Data/ContactModel/ COMPONENT data_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT data_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemDataContactModel PROPERTY PROJECT_LABEL "ContactModel")
	SET_PROPERTY(TARGET GolemDataContactModel PROPERTY FOLDER "Plugin/Data")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_DATA)
