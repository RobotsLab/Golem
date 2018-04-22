###############################################################################
#
# Golem Tools
#
###############################################################################

#option(GOLEM_BUILD_TOOLS "Build Golem Tools libraries" YES)

#IF (GOLEM_BUILD_TOOLS)
	SET(PACKAGE_TOOLS "${GOLEM_PROJECT_PACKAGES}/Tools")
	PACKAGE_LIST("Tools")
#ENDIF (GOLEM_BUILD_TOOLS)

###############################################################################
