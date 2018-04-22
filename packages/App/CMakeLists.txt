###############################################################################
#
# Golem App
#
###############################################################################

option(GOLEM_BUILD_APP "Build Golem applications" YES)

IF (GOLEM_BUILD_APP)
	SET(PACKAGE_APP "${GOLEM_PROJECT_PACKAGES}/App")
	PACKAGE_LIST("App")
ENDIF (GOLEM_BUILD_APP)

###############################################################################
#
# Golem App/DAQFT
#
###############################################################################

option(GOLEM_BUILD_APP_DAQFT "Build Golem DAQFT based applications" NO)

IF (GOLEM_BUILD_APP_DAQFT)
	SET(PACKAGE_APP_DAQFT "${GOLEM_PROJECT_PACKAGES}/App/DAQFT")
	PACKAGE_LIST("App/DAQFT")
ENDIF (GOLEM_BUILD_APP_DAQFT)

###############################################################################
#
# Golem App/Phys
#
###############################################################################

option(GOLEM_BUILD_APP_PHYS "Build Golem Phys based applications" NO)

IF (GOLEM_BUILD_APP_PHYS)
	SET(PACKAGE_APP_PHYS "${GOLEM_PROJECT_PACKAGES}/App/Phys")
	PACKAGE_LIST("App/Phys")
ENDIF (GOLEM_BUILD_APP_PHYS)

###############################################################################
#
# Golem App/Sim
#
###############################################################################

option(GOLEM_BUILD_APP_SIM "Build Golem Sim based applications" YES)

IF (GOLEM_BUILD_APP_SIM)
	SET(PACKAGE_APP_SIM "${GOLEM_PROJECT_PACKAGES}/App/Sim")
	PACKAGE_LIST("App/Sim")
ENDIF (GOLEM_BUILD_APP_SIM)

###############################################################################
#
# Golem App/SM
#
###############################################################################

option(GOLEM_BUILD_APP_SM "Build Golem SM based applications" YES)

IF (GOLEM_BUILD_APP_SM)
	SET(PACKAGE_APP_SM "${GOLEM_PROJECT_PACKAGES}/App/SM")
	PACKAGE_LIST("App/SM")
ENDIF (GOLEM_BUILD_APP_SM)

###############################################################################
#
# Golem App/Tiny
#
###############################################################################

option(GOLEM_BUILD_APP_TINY "Build Golem Tiny based applications" NO)

IF (GOLEM_BUILD_APP_TINY)
	SET(PACKAGE_APP_TINY "${GOLEM_PROJECT_PACKAGES}/App/Tiny")
	PACKAGE_LIST("App/Tiny")
ENDIF (GOLEM_BUILD_APP_TINY)

###############################################################################
#
# Golem App/UI
#
###############################################################################

option(GOLEM_BUILD_APP_UI "Build Golem UI based applications" YES)

IF (GOLEM_BUILD_APP_UI)
	SET(PACKAGE_APP_UI "${GOLEM_PROJECT_PACKAGES}/App/UI")
	PACKAGE_LIST("App/UI")
ENDIF (GOLEM_BUILD_APP_UI)

###############################################################################
