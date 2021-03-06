###############################################################################
#
# Golem UI/App library
#
###############################################################################

if(GOLEM_BUILD_UI)
	SET(PACKAGE_DIR "${PACKAGE_UI}/App")

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
	FIND_PACKAGE(Boost COMPONENTS filesystem)
	# OpenCV
	FIND_PACKAGE(OpenCV REQUIRED)

	FILE(GLOB_RECURSE PACKAGE_SOURCES "${PACKAGE_DIR}/src/*.cpp")
	FILE(GLOB_RECURSE PACKAGE_HEADERS "${PACKAGE_DIR}/include/*.h")

	if(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemApp SHARED ${PACKAGE_SOURCES} ${PACKAGE_HEADERS})
		if (WIN32)		
			TARGET_COMPILE_OPTIONS(GolemApp PUBLIC ${CMAKE_DLL_EXPORT_FLAGS})
		elseif (UNIX)
		endif()
	else(GOLEM_DYNAMIC_LIBS)
		ADD_LIBRARY(GolemApp STATIC ${PACKAGE_SOURCES} ${PACKAGE_HEADERS})
	endif(GOLEM_DYNAMIC_LIBS)

	if (WIN32)
		TARGET_LINK_LIBRARIES(GolemApp GolemTools GolemSim GolemSM ${Boost_LIBRARIES})
	elseif (UNIX)
		TARGET_LINK_LIBRARIES(GolemApp GolemTools GolemSim GolemSM GolemPlannerGraphPlanner ${Boost_LIBRARIES})
	endif()
	
	TARGET_INCLUDE_DIRECTORIES(GolemApp PUBLIC ${PACKAGE_DIR}/include ${GOLEM_PROJECT_PACKAGES}/Plugin/Planner/GraphPlanner/include ${GOLEM_PROJECT_PACKAGES}/Plugin/Ctrl/CtrlServer/include ${Boost_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
	
	if (CPACK_BUILD_CORE)
		if(GOLEM_DYNAMIC_LIBS)
			if (WIN32)
				INSTALL(TARGETS GolemApp RUNTIME DESTINATION bin COMPONENT core_libs)
			elseif (UNIX)
				INSTALL(TARGETS GolemApp LIBRARY DESTINATION bin COMPONENT core_libs)
			endif()
		else(GOLEM_DYNAMIC_LIBS)
			INSTALL(TARGETS GolemApp ARCHIVE DESTINATION lib COMPONENT core_libs)
		endif(GOLEM_DYNAMIC_LIBS)
		INSTALL(FILES ${PACKAGE_HEADERS} DESTINATION include/Golem/App/ COMPONENT core_headers)
	endif (CPACK_BUILD_CORE)
	
	SET_PROPERTY(TARGET GolemApp PROPERTY PROJECT_LABEL "App")
	SET_PROPERTY(TARGET GolemApp PROPERTY FOLDER "UI")
	SOURCE_GROUP("Include Files" FILES ${PACKAGE_HEADERS})
endif(GOLEM_BUILD_UI)
