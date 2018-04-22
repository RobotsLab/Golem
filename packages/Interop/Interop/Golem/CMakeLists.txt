###############################################################################
#
# Golem Interop/Interop/Golem library
#
###############################################################################

option(GOLEM_BUILD_INTEROP_GOLEM "Build Golem Interop Golem library" YES)
mark_as_advanced(GOLEM_BUILD_INTEROP_GOLEM)

if(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_GOLEM)
	SET(PACKAGE_DIR "${PACKAGE_INTEROP}/Interop/Golem")

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")

	if(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemInteropGolem SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS})
		if (WIN32)		
			TARGET_COMPILE_OPTIONS(GolemInteropGolem PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
		elseif (UNIX)
		endif()
	else(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemInteropGolem STATIC ${PACKAGE_SOURCES} ${PACKAGE_HEADERS})
	endif(GOLEM_DYNAMIC_LIBS)

	TARGET_LINK_LIBRARIES(GolemInteropGolem GolemInteropContact GolemContact)
	
	TARGET_INCLUDE_DIRECTORIES(GolemInteropGolem PUBLIC ${PACKAGE_DIR}/include PUBLIC ${GOLEM_PROJECT_PACKAGES}/Interop/Interop/Golem/include/Golem/Interop)
	
	if (CPACK_BUILD_CORE)
		if(GOLEM_DYNAMIC_LIBS)
			if (WIN32)
				INSTALL(TARGETS GolemInteropGolem RUNTIME DESTINATION bin COMPONENT core_libs)
			elseif (UNIX)
				INSTALL(TARGETS GolemInteropGolem LIBRARY DESTINATION bin COMPONENT core_libs)
			endif()
		else(GOLEM_DYNAMIC_LIBS)
			INSTALL(TARGETS GolemInteropGolem ARCHIVE DESTINATION lib COMPONENT core_libs)
		endif(GOLEM_DYNAMIC_LIBS)
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Interop/ COMPONENT core_headers)
	endif (CPACK_BUILD_CORE)
	
	SET_PROPERTY(TARGET GolemInteropGolem PROPERTY PROJECT_LABEL "Golem")
	SET_PROPERTY(TARGET GolemInteropGolem PROPERTY FOLDER "Interop/Interop")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
endif(GOLEM_BUILD_INTEROP AND GOLEM_BUILD_INTEROP_GOLEM)
