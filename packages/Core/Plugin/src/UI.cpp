/** @file UI.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Plugin/UI.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const char* OpenGL::frameName[FRAME_SIZE] = {
	"Disabled", "X", "Y", "Z", "Roll", "Pitch", "Yaw"
};

//------------------------------------------------------------------------------
