###############################################################################
#
# Golem Core
#
###############################################################################

option(GOLEM_BUILD_CORE "Build Golem Core libraries" YES)

IF (GOLEM_BUILD_CORE)
	SET(PACKAGE_CORE "${GOLEM_PROJECT_PACKAGES}/Core")
	PACKAGE_LIST("Core")
ENDIF (GOLEM_BUILD_CORE)

###############################################################################
