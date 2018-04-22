###############################################################################
#
# GraspCloudPlanning Master plugin
#
###############################################################################

option(GOLEM_BUILD_INTEROP_PLUGIN_GRASP_CLOUD_PLANNING "Build Golem Interop GraspCloudPlanning Master plugin" ON)
mark_as_advanced(GOLEM_BUILD_INTEROP_PLUGIN_GRASP_CLOUD_PLANNING)

if(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_PLUGIN AND GOLEM_BUILD_INTEROP_PLUGIN_GRASP_CLOUD_PLANNING)
	SET(PACKAGE_DIR "${PACKAGE_INTEROP_PLUGIN}/GraspCloudPlanning")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.config")
	FILE(GLOB_RECURSE PACKAGE_FILES_BIN "${PACKAGE_DIR}/resources/*.pcd")

	ADD_LIBRARY(GolemInteropPluginGraspCloudPlanning SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if(WIN32)
		SET(CMAKE_DLL_EXPORT_FLAGS "/DGOLEM_INTEROP_LIBRARY_DECLDIR_EXPORT")
		TARGET_COMPILE_OPTIONS(GolemInteropPluginGraspCloudPlanning PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif(UNIX)
		#TARGET_COMPILE_OPTIONS(GolemInteropPluginGraspCloudPlanning PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	endif()

	TARGET_LINK_LIBRARIES(GolemInteropPluginGraspCloudPlanning GolemInteropPCL)
	
	TARGET_INCLUDE_DIRECTORIES(GolemInteropPluginGraspCloudPlanning PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemInteropPluginGraspCloudPlanning ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})
	COPY_FILES(GolemInteropPluginGraspCloudPlanning ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES_BIN})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemInteropPluginGraspCloudPlanning RUNTIME DESTINATION bin COMPONENT core_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemInteropPluginGraspCloudPlanning LIBRARY DESTINATION bin COMPONENT core_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Interop/GraspCloudPlanning/ COMPONENT core_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT core_configs)
		INSTALL(FILES ${PACKAGE_FILES_BIN} DESTINATION bin COMPONENT core_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemInteropPluginGraspCloudPlanning PROPERTY PROJECT_LABEL "GraspCloudPlanning")
	SET_PROPERTY(TARGET GolemInteropPluginGraspCloudPlanning PROPERTY FOLDER "Interop/Plugin")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_PLUGIN AND GOLEM_BUILD_INTEROP_PLUGIN_GRASP_CLOUD_PLANNING)

###############################################################################
