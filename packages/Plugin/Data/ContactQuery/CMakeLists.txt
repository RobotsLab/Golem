###############################################################################
#
# Golem Plugin/Data/ContactQuery library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_DATA)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_DATA}/ContactQuery")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemDataContactQuery SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	
	TARGET_COMPILE_OPTIONS(GolemDataContactQuery PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	
	TARGET_INCLUDE_DIRECTORIES(GolemDataContactQuery PUBLIC ${PACKAGE_DIR}/include)

	TARGET_LINK_LIBRARIES(GolemDataContactQuery GolemContact)
	
	COPY_FILES(GolemDataContactQuery ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemDataContactQuery RUNTIME DESTINATION bin COMPONENT data_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemDataContactQuery LIBRARY DESTINATION bin COMPONENT data_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Data/ContactQuery/ COMPONENT data_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT data_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemDataContactQuery PROPERTY PROJECT_LABEL "ContactQuery")
	SET_PROPERTY(TARGET GolemDataContactQuery PROPERTY FOLDER "Plugin/Data")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_DATA)
