###############################################################################
#
# Golem Interop
#
###############################################################################

option(GOLEM_BUILD_INTEROP "Build Golem Interop" YES)

IF (GOLEM_BUILD_INTEROP)
	SET(PACKAGE_INTEROP "${GOLEM_PROJECT_PACKAGES}/Interop")
	PACKAGE_LIST("Interop/Interop")
	PACKAGE_LIST("Interop")
ENDIF (GOLEM_BUILD_INTEROP)

###############################################################################
#
# Golem Interop/Plugin
#
###############################################################################

option(GOLEM_BUILD_INTEROP_PLUGIN "Build Golem Interop Plugin" NO)

IF (GOLEM_BUILD_INTEROP_PLUGIN)
	SET(PACKAGE_INTEROP_PLUGIN "${GOLEM_PROJECT_PACKAGES}/Interop/Plugin")
	PACKAGE_LIST("Interop/Plugin")
ENDIF (GOLEM_BUILD_INTEROP_PLUGIN)

###############################################################################
