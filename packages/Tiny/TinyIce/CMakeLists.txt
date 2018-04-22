###############################################################################
#
# Golem Tiny/TinyIce server
#
###############################################################################

if(GOLEM_BUILD_TINY)
	SET(PACKAGE_DIR "${PACKAGE_TINY}/TinyIce")

	find_package(Ice REQUIRED Ice IceUtil)

	#FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	#FILE(GLOB_RECURSE PACKAGE_SOURCES_ICE "${PACKAGE_DIR}/include/*.cc")
	#FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")
	#FILE(GLOB_RECURSE PACKAGE_HEADERS_ICE "${PACKAGE_DIR}/include/*.hh")
	SET(PACKAGE_SOURCES ${PACKAGE_DIR}/src/TinyIceI.cpp)
	SET(PACKAGE_SOURCES_ICE ${PACKAGE_DIR}/include/Golem/TinyIce/TinyIce.cc)
	SET(PACKAGE_HEADERS ${PACKAGE_DIR}/include/Golem/TinyIce/TinyIceI.h)
	SET(PACKAGE_HEADERS_ICE ${PACKAGE_DIR}/include/Golem/TinyIce/TinyIce.hh)

	FILE(GLOB_RECURSE PACKAGE_FILES "${PACKAGE_DIR}/resources/*.xml")
	FILE(GLOB_RECURSE PACKAGE_FILES_ICE "${PACKAGE_DIR}/resources/*.ice")

	ADD_CUSTOM_COMMAND(
		OUTPUT ${PACKAGE_DIR}/include/Golem/TinyIce/TinyIce.cc ${PACKAGE_DIR}/include/Golem/TinyIce/TinyIce.hh
		COMMAND ${Ice_SLICE2CPP_EXECUTABLE} --header-ext hh --source-ext cc --output-dir ${PACKAGE_DIR}/include/Golem/TinyIce --include-dir Golem/TinyIce ${PACKAGE_DIR}/resources/TinyIce.ice
		DEPENDS ${PACKAGE_DIR}/resources/TinyIce.ice
		COMMENT "slice2cpp"
	)
	
	ADD_EXECUTABLE(GolemTinyIce ${PACKAGE_SOURCES} ${PACKAGE_SOURCES_ICE} ${PACKAGE_HEADERS} ${PACKAGE_FILES} ${PACKAGE_FILES_ICE})
	#ADD_CUSTOM_COMMAND(
	#	TARGET GolemTinyIce
	#	PRE_BUILD
	#	COMMAND ${Ice_SLICE2CPP_EXECUTABLE} --header-ext hh --source-ext cc --output-dir ${PACKAGE_DIR}/include/Golem/TinyIce --include-dir Golem/TinyIce ${PACKAGE_DIR}/resources/TinyIce.ice
	#	WORKING_DIRECTORY ${PACKAGE_DIR}/include/Golem/TinyIce
	#	COMMENT "slice2cpp"
	#)
	TARGET_LINK_LIBRARIES(GolemTinyIce GolemSim GolemApp GolemPhys GolemCtrlKatana ${Ice_LIBRARIES})
	if(WIN32)
		TARGET_COMPILE_OPTIONS(GolemTinyIce PUBLIC /wd4091)
	elseif(UNIX)
	endif()
	TARGET_INCLUDE_DIRECTORIES(GolemTinyIce PUBLIC ${PACKAGE_DIR}/include ${Ice_INCLUDE_DIRS})

	COPY_FILES(GolemTinyIce ${RUNTIME_OUTPUT_DIRECTORY} ${PACKAGE_FILES})
	SET_PROPERTY(TARGET GolemTinyIce PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
	SET_PROPERTY(TARGET GolemTinyIce PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
	
	if (CPACK_BUILD_TINY)
		INSTALL(TARGETS GolemTinyIce RUNTIME DESTINATION bin COMPONENT tiny_execs)
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/TinyIce/ COMPONENT tiny_headers)
		INSTALL(FILES ${PACKAGE_HEADERS_ICE} DESTINATION include/Golem/TinyIce/ COMPONENT tiny_headers)
		INSTALL(FILES ${PACKAGE_FILES} DESTINATION bin COMPONENT tiny_configs)
	endif (CPACK_BUILD_TINY)

	SET_PROPERTY(TARGET GolemTinyIce PROPERTY PROJECT_LABEL "TinyIce")
	SET_PROPERTY(TARGET GolemTinyIce PROPERTY FOLDER "Tiny")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${PACKAGE_FILES} ${PACKAGE_FILES_ICE})
endif(GOLEM_BUILD_TINY)
