###############################################################################
#
# GolemContact plugin
#
###############################################################################

option(GOLEM_BUILD_INTEROP_PLUGIN_GOLEM_CONTACT "Build Golem Interop GolemContact plugin" ON)
mark_as_advanced(GOLEM_BUILD_INTEROP_PLUGIN_GOLEM_CONTACT)

if(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_PLUGIN AND GOLEM_BUILD_INTEROP_PLUGIN_GOLEM_CONTACT)
	SET(PACKAGE_DIR "${PACKAGE_INTEROP_PLUGIN}/GolemContact")

	# OpenCV
	FIND_PACKAGE(OpenCV REQUIRED)

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")
	FILE(GLOB_RECURSE PACKAGE_FILES_BIN "${PACKAGE_DIR}/resources/*.model")

	ADD_LIBRARY(GolemInteropPluginGolemContact SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if(WIN32)
		SET(CMAKE_DLL_EXPORT_FLAGS "/DGOLEM_INTEROP_LIBRARY_DECLDIR_EXPORT")
		TARGET_COMPILE_OPTIONS(GolemInteropPluginGolemContact PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif(UNIX)
		#TARGET_COMPILE_OPTIONS(GolemInteropPluginGolemContact PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	endif()

	TARGET_LINK_LIBRARIES(GolemInteropPluginGolemContact GolemInteropContact GolemInteropGolem GolemApp GolemDataImage GolemDataFeature3D ${OpenCV_LIBS})
	
	TARGET_COMPILE_DEFINITIONS(GolemInteropPluginGolemContact PUBLIC ${OPENCV_DEFINITIONS})

	TARGET_INCLUDE_DIRECTORIES(GolemInteropPluginGolemContact PUBLIC ${PACKAGE_DIR}/include PUBLIC ${OPENCV_INCLUDE_DIRS})

	COPY_FILES(GolemInteropPluginGolemContact ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})
	COPY_FILES(GolemInteropPluginGolemContact ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES_BIN})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemInteropPluginGolemContact RUNTIME DESTINATION bin COMPONENT core_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemInteropPluginGolemContact LIBRARY DESTINATION bin COMPONENT core_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Interop/GolemContact/ COMPONENT core_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT core_configs)
		INSTALL(FILES ${PACKAGE_FILES_BIN} DESTINATION bin COMPONENT core_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemInteropPluginGolemContact PROPERTY PROJECT_LABEL "GolemContact")
	SET_PROPERTY(TARGET GolemInteropPluginGolemContact PROPERTY FOLDER "Interop/Plugin")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_PLUGIN AND GOLEM_BUILD_INTEROP_PLUGIN_GOLEM_CONTACT)

###############################################################################
