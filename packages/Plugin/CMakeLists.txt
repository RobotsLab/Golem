###############################################################################
#
# Golem Plugin/ActiveCtrl
#
###############################################################################

option(GOLEM_BUILD_PLUGIN_ACTIVECTRL "Build Golem Plugin/ActiveCtrl plugins" YES)

IF (GOLEM_BUILD_PLUGIN_ACTIVECTRL)
	SET(PACKAGE_PLUGIN_ACTIVECTRL "${GOLEM_PROJECT_PACKAGES}/Plugin/ActiveCtrl")
	PACKAGE_LIST("Plugin/ActiveCtrl")
ENDIF (GOLEM_BUILD_PLUGIN_ACTIVECTRL)

###############################################################################
#
# Golem Plugin/Ctrl
#
###############################################################################

option(GOLEM_BUILD_PLUGIN_CTRL "Build Golem Plugin/Ctrl plugins" YES)

IF (GOLEM_BUILD_PLUGIN_CTRL)
	SET(PACKAGE_PLUGIN_CTRL "${GOLEM_PROJECT_PACKAGES}/Plugin/Ctrl")
	PACKAGE_LIST("Plugin/Ctrl")
ENDIF (GOLEM_BUILD_PLUGIN_CTRL)

###############################################################################
#
# Golem Plugin/Data
#
###############################################################################

option(GOLEM_BUILD_PLUGIN_DATA "Build Golem Plugin/Data plugins" YES)

IF (GOLEM_BUILD_PLUGIN_DATA)
	SET(PACKAGE_PLUGIN_DATA "${GOLEM_PROJECT_PACKAGES}/Plugin/Data")
	PACKAGE_LIST("Plugin/Data")
ENDIF (GOLEM_BUILD_PLUGIN_DATA)

###############################################################################
#
# Golem Plugin/Planner
#
###############################################################################

option(GOLEM_BUILD_PLUGIN_PLANNER "Build Golem Plugin/Planner plugins" YES)

IF (GOLEM_BUILD_PLUGIN_PLANNER)
	SET(PACKAGE_PLUGIN_PLANNER "${GOLEM_PROJECT_PACKAGES}/Plugin/Planner")
	PACKAGE_LIST("Plugin/Planner")
ENDIF (GOLEM_BUILD_PLUGIN_PLANNER)

###############################################################################
#
# Golem Plugin/Sensor
#
###############################################################################

option(GOLEM_BUILD_PLUGIN_SENSOR "Build Golem Plugin/Sensor plugins" YES)

IF (GOLEM_BUILD_PLUGIN_SENSOR)
	SET(PACKAGE_PLUGIN_SENSOR "${GOLEM_PROJECT_PACKAGES}/Plugin/Sensor")
	PACKAGE_LIST("Plugin/Sensor")
ENDIF (GOLEM_BUILD_PLUGIN_SENSOR)

###############################################################################
