###############################################################################
#
# Golem Interop/Interop/Core library
#
###############################################################################

if(GOLEM_BUILD_INTEROP)
	SET(PACKAGE_DIR "${PACKAGE_INTEROP}/Interop/Core")

	# Boost
	if (WIN32)
		ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
		SET(Boost_USE_STATIC_LIBS ON)
		SET(Boost_ALL_DYN_LINK OFF)
	elseif (UNIX)
		#ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
		#SET(Boost_USE_STATIC_LIBS OFF)
		#SET(Boost_ALL_DYN_LINK ON)
	endif ()
	FIND_PACKAGE(Boost COMPONENTS system filesystem)

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")

	if(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemInteropCore SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS})
		if (WIN32)		
			TARGET_COMPILE_OPTIONS(GolemInteropCore PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
		elseif (UNIX)
		endif()
	else(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemInteropCore STATIC ${PACKAGE_SOURCES} ${PACKAGE_HEADERS})
	endif(GOLEM_DYNAMIC_LIBS)

	if (WIN32)
		TARGET_LINK_LIBRARIES(GolemInteropCore ${Boost_LIBRARIES})
	elseif (UNIX)
		TARGET_LINK_LIBRARIES(GolemInteropCore ${Boost_LIBRARIES} ${CMAKE_DL_LIBS})
	endif ()
	
	TARGET_INCLUDE_DIRECTORIES(GolemInteropCore PUBLIC ${PACKAGE_DIR}/include PUBLIC ${GOLEM_PROJECT_PACKAGES}/Interop/Interop/Core/include/Golem/Interop PUBLIC ${Boost_INCLUDE_DIRS})
	
	if (CPACK_BUILD_CORE)
		if(GOLEM_DYNAMIC_LIBS)
			if (WIN32)
				INSTALL(TARGETS GolemInteropCore RUNTIME DESTINATION bin COMPONENT core_libs)
			elseif (UNIX)
				INSTALL(TARGETS GolemInteropCore LIBRARY DESTINATION bin COMPONENT core_libs)
			endif()
		else(GOLEM_DYNAMIC_LIBS)
			INSTALL(TARGETS GolemInteropCore ARCHIVE DESTINATION lib COMPONENT core_libs)
		endif(GOLEM_DYNAMIC_LIBS)
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/Interop/ COMPONENT core_headers)
	endif (CPACK_BUILD_CORE)
	
	SET_PROPERTY(TARGET GolemInteropCore PROPERTY PROJECT_LABEL "Core")
	SET_PROPERTY(TARGET GolemInteropCore PROPERTY FOLDER "Interop/Interop")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
endif(GOLEM_BUILD_INTEROP)
