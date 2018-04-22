###############################################################################
#
# Golem Plugin/Data/ContactQueryUA library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_DATA)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_DATA}/ContactQueryUA")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemDataContactQueryUA SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if(WIN32)
		TARGET_COMPILE_OPTIONS(GolemDataContactQueryUA PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif(UNIX)
		TARGET_COMPILE_OPTIONS(GolemDataContactQueryUA PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	endif()

	TARGET_LINK_LIBRARIES(GolemDataContactQueryUA GolemDataContactQuery)
	
	TARGET_INCLUDE_DIRECTORIES(GolemDataContactQueryUA PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemDataContactQueryUA ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemDataContactQueryUA RUNTIME DESTINATION bin COMPONENT data_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemDataContactQueryUA LIBRARY DESTINATION bin COMPONENT data_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Data/ContactQueryUA/ COMPONENT data_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT data_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemDataContactQueryUA PROPERTY PROJECT_LABEL "ContactQueryUA")
	SET_PROPERTY(TARGET GolemDataContactQueryUA PROPERTY FOLDER "Plugin/Data")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_DATA)
