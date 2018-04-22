# - Find PCAN
# Find the native PCAN headers and libraries.
#
#  PCAN_INCLUDE_DIR -  where to find the include files of PCAN
#  PCAN_LIBRARIES    - List of libraries when using PCAN.
#  PCAN_FOUND        - True if PCAN found.

GET_FILENAME_COMPONENT(module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH)

IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
	SET(LIB "x64")
ELSE(CMAKE_SIZEOF_VOID_P EQUAL 8)
	SET(LIB "Win32")
ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8)

SET(program_files_path "$ENV{PROGRAMFILES}")

IF( program_files_path )
	FILE(TO_CMAKE_PATH $ENV{PROGRAMFILES} program_files_path )
ENDIF( program_files_path )

# Look for the header file.
SET(PCAN_INCLUDE_PATHS
	/usr/include
	/usr/local/include
	"${program_files_path}/PCAN/Include/C++"
	"C:/Program Files (x86)/PCAN/Include/C++"
	"C:/Program Files/PCAN/Include/C++"
)
if(WIN32)
	FIND_PATH(PCAN_INCLUDE NAMES pcan_usb.h PATHS ${PCAN_INCLUDE_PATHS})
elseif(UNIX)
	FIND_PATH(PCAN_INCLUDE NAMES libpcan.h PATHS ${PCAN_INCLUDE_PATHS})
endif()
MARK_AS_ADVANCED(PCAN_INCLUDE)

# Look for the library.
SET(PCAN_LIBRARY_PATHS
	/usr/lib
	/usr/local/lib
	"${program_files_path}/PCAN/${LIB}/VC_LIB"
	"C:/Program Files (x86)/PCAN/${LIB}/VC_LIB"
	"C:/Program Files/PCAN/${LIB}/VC_LIB"
)
if(WIN32)
	FIND_LIBRARY(PCAN_LIBRARY NAMES PCAN_USB PATHS ${PCAN_LIBRARY_PATHS})
elseif(UNIX)
	FIND_LIBRARY(PCAN_LIBRARY NAMES pcan PATHS ${PCAN_LIBRARY_PATHS})
endif()
MARK_AS_ADVANCED(PCAN_LIBRARY)

# Copy the results to the output variables.
IF(PCAN_INCLUDE AND PCAN_LIBRARY)
	SET(PCAN_FOUND 1)
	SET(PCAN_LIBRARIES ${PCAN_LIBRARY})
	SET(PCAN_INCLUDE_DIR ${PCAN_INCLUDE})
ELSE(PCAN_INCLUDE AND PCAN_LIBRARY)
	SET(PCAN_FOUND 0)
	SET(PCAN_LIBRARIES)
	SET(PCAN_INCLUDE_DIR)
ENDIF(PCAN_INCLUDE AND PCAN_LIBRARY)

# Report the results.
IF(NOT PCAN_FOUND)
	SET(PCAN_DIR_MESSAGE
		"PCAN was not found. Make sure PCAN_LIBRARIES and PCAN_INCLUDE_DIR are set.")
	IF(PCAN_FIND_REQUIRED)
		MESSAGE(FATAL_ERROR "${PCAN_DIR_MESSAGE}")
	ELSEIF(NOT PCAN_FIND_QUIETLY)
		MESSAGE(STATUS "${PCAN_DIR_MESSAGE}")
	ENDIF(PCAN_FIND_REQUIRED)
ENDIF(NOT PCAN_FOUND)
