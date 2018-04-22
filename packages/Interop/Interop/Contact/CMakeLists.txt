###############################################################################
#
# Golem Interop/Interop/Contact library
#
###############################################################################

option(GOLEM_BUILD_INTEROP_CONTACT "Build Golem Interop Contact library" YES)
mark_as_advanced(GOLEM_BUILD_INTEROP_CONTACT)

if(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_CONTACT)
	SET(PACKAGE_DIR "${PACKAGE_INTEROP}/Interop/Contact")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")

	if(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemInteropContact SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS})
		if (WIN32)		
			TARGET_COMPILE_OPTIONS(GolemInteropContact PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
		elseif (UNIX)
		endif()
	else(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemInteropContact STATIC ${PACKAGE_SOURCES} ${PACKAGE_HEADERS})
	endif(GOLEM_DYNAMIC_LIBS)

	TARGET_LINK_LIBRARIES(GolemInteropContact GolemInteropCore GolemInteropPCL)
	
	TARGET_INCLUDE_DIRECTORIES(GolemInteropContact PUBLIC ${PACKAGE_DIR}/include PUBLIC ${GOLEM_PROJECT_PACKAGES}/Interop/Interop/Contact/include/Golem/Interop)
	
	if (CPACK_BUILD_CORE)
		if(GOLEM_DYNAMIC_LIBS)
			if (WIN32)
				INSTALL(TARGETS GolemInteropContact RUNTIME DESTINATION bin COMPONENT core_libs)
			elseif (UNIX)
				INSTALL(TARGETS GolemInteropContact LIBRARY DESTINATION bin COMPONENT core_libs)
			endif()
		else(GOLEM_DYNAMIC_LIBS)
			INSTALL(TARGETS GolemInteropContact ARCHIVE DESTINATION lib COMPONENT core_libs)
		endif(GOLEM_DYNAMIC_LIBS)
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Interop/ COMPONENT core_headers)
	endif (CPACK_BUILD_CORE)
	
	SET_PROPERTY(TARGET GolemInteropContact PROPERTY PROJECT_LABEL "Contact")
	SET_PROPERTY(TARGET GolemInteropContact PROPERTY FOLDER "Interop/Interop")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
endif(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_CONTACT)
