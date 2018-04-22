###############################################################################
#
# Golem Plugin/Ctrl/KukaKR5Sixx library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_CTRL)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_CTRL}/KukaKR5Sixx")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	#FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	SET(PACKAGE_HEADERS
		${PACKAGE_DIR}/include/Golem/Ctrl/KukaKR5Sixx/KukaKR5Sixx.h
		${PACKAGE_DIR}/include/Golem/Ctrl/KukaKR5Sixx/Data.h
	)
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemCtrlKukaKR5Sixx SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if(WIN32)
		TARGET_COMPILE_OPTIONS(GolemCtrlKukaKR5Sixx PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif(UNIX)
		#TARGET_COMPILE_OPTIONS(GolemCtrlKukaKR5Sixx PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	endif()

	TARGET_LINK_LIBRARIES(GolemCtrlKukaKR5Sixx GolemCtrlKuka GolemCtrlSingleCtrl)
	
	TARGET_INCLUDE_DIRECTORIES(GolemCtrlKukaKR5Sixx PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemCtrlKukaKR5Sixx ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemCtrlKukaKR5Sixx RUNTIME DESTINATION bin COMPONENT ctrl_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemCtrlKukaKR5Sixx LIBRARY DESTINATION bin COMPONENT ctrl_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Ctrl/KukaKR5Sixx/ COMPONENT ctrl_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT ctrl_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemCtrlKukaKR5Sixx PROPERTY PROJECT_LABEL "KukaKR5Sixx")
	SET_PROPERTY(TARGET GolemCtrlKukaKR5Sixx PROPERTY FOLDER "Plugin/Ctrl")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_CTRL)
