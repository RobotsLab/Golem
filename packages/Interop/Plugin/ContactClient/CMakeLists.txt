###############################################################################
#
# ContactClient plugin
#
###############################################################################

option(GOLEM_BUILD_INTEROP_PLUGIN_CONTACT_CLIENT "Build Golem Interop ContactClient plugin" ON)
mark_as_advanced(GOLEM_BUILD_INTEROP_PLUGIN_CONTACT_CLIENT)

if(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_PLUGIN AND GOLEM_BUILD_INTEROP_PLUGIN_CONTACT_CLIENT)
	SET(PACKAGE_DIR "${PACKAGE_INTEROP_PLUGIN}/ContactClient")

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

	ADD_LIBRARY(GolemInteropPluginContactClient SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if(WIN32)
		SET(CMAKE_DLL_EXPORT_FLAGS "/DGOLEM_INTEROP_LIBRARY_DECLDIR_EXPORT")
		TARGET_COMPILE_OPTIONS(GolemInteropPluginContactClient PUBLIC ${CMAKE_DLL_EXPORT_FLAGS} /wd4005)
	elseif(UNIX)
		#TARGET_COMPILE_OPTIONS(GolemInteropPluginContactClient PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	endif()

	TARGET_LINK_LIBRARIES(GolemInteropPluginContactClient GolemInteropContact ${Boost_LIBRARIES})
	
	TARGET_INCLUDE_DIRECTORIES(GolemInteropPluginContactClient PUBLIC ${PACKAGE_DIR}/include PUBLIC ${Boost_INCLUDE_DIRS})

	COPY_FILES(GolemInteropPluginContactClient ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemInteropPluginContactClient RUNTIME DESTINATION bin COMPONENT core_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemInteropPluginContactClient LIBRARY DESTINATION bin COMPONENT core_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Interop/ContactClient/ COMPONENT core_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT core_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemInteropPluginContactClient PROPERTY PROJECT_LABEL "ContactClient")
	SET_PROPERTY(TARGET GolemInteropPluginContactClient PROPERTY FOLDER "Interop/Plugin")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_PLUGIN AND GOLEM_BUILD_INTEROP_PLUGIN_CONTACT_CLIENT)

###############################################################################
