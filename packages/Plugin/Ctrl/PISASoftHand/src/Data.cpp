/** @file Data.cpp
 *
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Ctrl/PISASoftHand/Data.h>
#include <Golem/Ctrl/PISA/Data.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(PISASoftHandSerial::Desc &val, XMLContext* context, bool create) {
	XMLData((PISASoftHand::Desc&)val, context, create);

#ifdef WIN32
    XMLData("serial_port_windows", val.serialPort, context->getContextFirst("pisa"), create);
#else
	XMLData("serial_port_linux", val.serialPort, context->getContextFirst("pisa"), create);
#endif
	XMLData("motor_file", val.motorFile, context->getContextFirst("pisa"), create);
}

//------------------------------------------------------------------------------
