###############################################################################
#
# Golem Core/Contact library
#
###############################################################################

if(GOLEM_BUILD_CORE)
	SET(PACKAGE_DIR "${PACKAGE_CORE}/Contact")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")

	if(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemContact SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS})		
		TARGET_COMPILE_OPTIONS(GolemContact PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
	else(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemContact STATIC ${PACKAGE_SOURCES} ${PACKAGE_HEADERS})
	endif(GOLEM_DYNAMIC_LIBS)

	# Windows 32/64 bit
	if (WIN32)
		TARGET_COMPILE_OPTIONS(GolemContact PUBLIC /wd4244)
	# Unix Linux/Apple
	elseif (UNIX)
	endif ()

	TARGET_LINK_LIBRARIES(GolemContact GolemTools)

	TARGET_INCLUDE_DIRECTORIES(GolemContact PUBLIC ${PACKAGE_DIR}/include)
		
	if (CPACK_BUILD_CORE)
		if(GOLEM_DYNAMIC_LIBS)
			if (WIN32)
				INSTALL(TARGETS GolemContact RUNTIME DESTINATION bin COMPONENT core_libs)
			elseif (UNIX)
				INSTALL(TARGETS GolemContact LIBRARY DESTINATION bin COMPONENT core_libs)
			endif()
		else(GOLEM_DYNAMIC_LIBS)
			INSTALL(TARGETS GolemContact ARCHIVE DESTINATION lib COMPONENT core_libs)
		endif(GOLEM_DYNAMIC_LIBS)
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Contact/ COMPONENT core_headers)
	endif (CPACK_BUILD_CORE)
	
	SET_PROPERTY(TARGET GolemContact PROPERTY PROJECT_LABEL "Contact")
	SET_PROPERTY(TARGET GolemContact PROPERTY FOLDER "Core")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
endif(GOLEM_BUILD_CORE)
