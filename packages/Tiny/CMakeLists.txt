###############################################################################
#
# Golem Tiny
#
###############################################################################

option(GOLEM_BUILD_TINY "Build Golem Tiny libraries" NO)

IF (GOLEM_BUILD_TINY)
	SET(PACKAGE_TINY "${GOLEM_PROJECT_PACKAGES}/Tiny")
	PACKAGE_LIST("Tiny")
ENDIF (GOLEM_BUILD_TINY)

###############################################################################
