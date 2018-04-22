###############################################################################
#
# Golem UI
#
###############################################################################

option(GOLEM_BUILD_UI "Build Golem UI libraries" YES)

IF (GOLEM_BUILD_UI)
	SET(PACKAGE_UI "${GOLEM_PROJECT_PACKAGES}/UI")
	PACKAGE_LIST("UI")
ENDIF (GOLEM_BUILD_UI)

###############################################################################
