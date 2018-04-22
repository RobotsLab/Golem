###############################################################################
#
# ContactServer plugin
#
###############################################################################

option(GOLEM_BUILD_INTEROP_PLUGIN_CONTACT_SERVER "Build Golem Interop ContactServer plugin" ON)
mark_as_advanced(GOLEM_BUILD_INTEROP_PLUGIN_CONTACT_SERVER)

if(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_PLUGIN AND GOLEM_BUILD_INTEROP_PLUGIN_CONTACT_SERVER)
	SET(PACKAGE_DIR "${PACKAGE_INTEROP_PLUGIN}/ContactServer")

	# Boost
	if (WIN32)
		ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
		SET(Boost_USE_STATIC_LIBS ON)
		SET(Boost_ALL_DYN_LINK OFF)
	elseif (UNIX)
		#ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
		#SET(Boost_USE_STATIC_LIBS OFF)
		#SET(Boost_ALL_DYN_LINK ON)
	endif ()
	FIND_PACKAGE(Boost COMPONENTS system)

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.config")

	ADD_LIBRARY(GolemInteropPluginContactServer SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if(WIN32)
		SET(CMAKE_DLL_EXPORT_FLAGS "/DGOLEM_INTEROP_LIBRARY_DECLDIR_EXPORT")
		TARGET_COMPILE_OPTIONS(GolemInteropPluginContactServer PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif(UNIX)
		#TARGET_COMPILE_OPTIONS(GolemInteropPluginContactServer PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	endif()

	TARGET_LINK_LIBRARIES(GolemInteropPluginContactServer GolemInteropPluginContactClient ${Boost_LIBRARIES})
	
	TARGET_INCLUDE_DIRECTORIES(GolemInteropPluginContactServer PUBLIC ${PACKAGE_DIR}/include PUBLIC ${Boost_INCLUDE_DIRS})

	COPY_FILES(GolemInteropPluginContactServer ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemInteropPluginContactServer RUNTIME DESTINATION bin COMPONENT core_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemInteropPluginContactServer LIBRARY DESTINATION bin COMPONENT core_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Interop/ContactServer/ COMPONENT core_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT core_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemInteropPluginContactServer PROPERTY PROJECT_LABEL "ContactServer")
	SET_PROPERTY(TARGET GolemInteropPluginContactServer PROPERTY FOLDER "Interop/Plugin")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_PLUGIN AND GOLEM_BUILD_INTEROP_PLUGIN_CONTACT_SERVER)

###############################################################################
