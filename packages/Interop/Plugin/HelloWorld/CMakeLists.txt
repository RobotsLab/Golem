###############################################################################
#
# HelloWorld Master plugin
#
###############################################################################

option(GOLEM_BUILD_INTEROP_PLUGIN_HELLO_WORLD "Build Golem Interop HelloWorld Master plugin" ON)
mark_as_advanced(GOLEM_BUILD_INTEROP_PLUGIN_HELLO_WORLD)

if(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_PLUGIN AND GOLEM_BUILD_INTEROP_PLUGIN_HELLO_WORLD)
	SET(PACKAGE_DIR "${PACKAGE_INTEROP_PLUGIN}/HelloWorld")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.config")

	ADD_LIBRARY(GolemInteropPluginHelloWorld SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if(WIN32)
		SET(CMAKE_DLL_EXPORT_FLAGS "/DGOLEM_INTEROP_LIBRARY_DECLDIR_EXPORT")
		TARGET_COMPILE_OPTIONS(GolemInteropPluginHelloWorld PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif(UNIX)
		#TARGET_COMPILE_OPTIONS(GolemInteropPluginHelloWorld PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	endif()

	TARGET_LINK_LIBRARIES(GolemInteropPluginHelloWorld GolemInteropCore)
	
	TARGET_INCLUDE_DIRECTORIES(GolemInteropPluginHelloWorld PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemInteropPluginHelloWorld ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemInteropPluginHelloWorld RUNTIME DESTINATION bin COMPONENT core_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemInteropPluginHelloWorld LIBRARY DESTINATION bin COMPONENT core_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Interop/HelloWorld/ COMPONENT core_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT core_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemInteropPluginHelloWorld PROPERTY PROJECT_LABEL "HelloWorld")
	SET_PROPERTY(TARGET GolemInteropPluginHelloWorld PROPERTY FOLDER "Interop/Plugin")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_PLUGIN AND GOLEM_BUILD_INTEROP_PLUGIN_HELLO_WORLD)

###############################################################################
