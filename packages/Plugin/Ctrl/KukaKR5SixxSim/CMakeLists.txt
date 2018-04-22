###############################################################################
#
# Golem Plugin/Ctrl/KukaKR5SixxSim library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_CTRL)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_CTRL}/KukaKR5SixxSim")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemCtrlKukaKR5SixxSim SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if (WIN32)		
		TARGET_COMPILE_OPTIONS(GolemCtrlKukaKR5SixxSim PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif (UNIX)
	endif()	

	TARGET_LINK_LIBRARIES(GolemCtrlKukaKR5SixxSim GolemCtrlKuka)
	
	TARGET_INCLUDE_DIRECTORIES(GolemCtrlKukaKR5SixxSim PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemCtrlKukaKR5SixxSim ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemCtrlKukaKR5SixxSim RUNTIME DESTINATION bin COMPONENT ctrl_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemCtrlKukaKR5SixxSim LIBRARY DESTINATION bin COMPONENT ctrl_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Ctrl/KukaKR5SixxSim/ COMPONENT ctrl_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT ctrl_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemCtrlKukaKR5SixxSim PROPERTY PROJECT_LABEL "KukaKR5SixxSim")
	SET_PROPERTY(TARGET GolemCtrlKukaKR5SixxSim PROPERTY FOLDER "Plugin/Ctrl")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_CTRL)
