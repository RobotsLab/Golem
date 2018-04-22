###############################################################################
#
# Golem Plugin/ActiveCtrl/ArmHandForce library
#
###############################################################################

if(GOLEM_BUILD_PLUGIN_ACTIVECTRL)
	SET(PACKAGE_DIR "${PACKAGE_PLUGIN_ACTIVECTRL}/ArmHandForce")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")

	ADD_LIBRARY(GolemArmHandForce SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS} ${PACKAGE_FILES})
	if(WIN32)
		TARGET_COMPILE_OPTIONS(GolemArmHandForce PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	elseif(UNIX)
		TARGET_COMPILE_OPTIONS(GolemArmHandForce PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	endif()

	TARGET_LINK_LIBRARIES(GolemArmHandForce GolemActiveCtrl GolemTools)
	
	TARGET_INCLUDE_DIRECTORIES(GolemArmHandForce PUBLIC ${PACKAGE_DIR}/include)

	COPY_FILES(GolemArmHandForce ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})

	if (CPACK_BUILD_CORE)
		if (WIN32)
			INSTALL(TARGETS GolemArmHandForce RUNTIME DESTINATION bin COMPONENT activectrl_execs)
		elseif (UNIX)
			INSTALL(TARGETS GolemArmHandForce LIBRARY DESTINATION bin COMPONENT activectrl_execs)
		endif()
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/ActiveCtrl/ArmHandForce/ COMPONENT activectrl_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT activectrl_configs)
	endif (CPACK_BUILD_CORE)

	SET_PROPERTY(TARGET GolemArmHandForce PROPERTY PROJECT_LABEL "ArmHandForce")
	SET_PROPERTY(TARGET GolemArmHandForce PROPERTY FOLDER "Plugin/ActiveCtrl")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES})
endif(GOLEM_BUILD_PLUGIN_ACTIVECTRL)
