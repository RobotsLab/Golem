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

#include <Golem/Ctrl/SingleCtrl/Data.h>
#include <Golem/Ctrl/Data.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLDataIO(SingleCtrl::Desc &val, XMLContext* context, bool create) {
	golem::XMLData("thread_priority", val.threadPriority, context, create);
	golem::XMLData("thread_timeout", val.threadTimeOut, context, create);
	golem::XMLData("state_queue_size", val.qStateSize, context, create);
	golem::XMLData("command_queue_size", val.qCommandSize, context, create);
	golem::XMLData("time_quant", val.timeQuant, context, create);

	golem::XMLData("cycle_avr_len", val.cycleLengthAverage, context, create);
	golem::XMLData("cycle_dur_ctrl", val.cycleDurationCtrl, context, create);
	golem::XMLData("cycle_dur_init", val.cycleDurationInit, context, create);
	golem::XMLData("cycle_dur_offs", val.cycleDurationOffs, context, create);
	golem::XMLData("cycle_dur_max_dev", val.cycleDurationMaxDev, context, create);

	try {
		golem::XMLData("cycle_cont_ctrl", val.cycleContinuousCtrl, context, create);
	}
	catch (const MsgXMLParserAttributeNotFound&) {
	}
}

void golem::XMLDataSim(SingleCtrl::Desc &val, XMLContext* context, bool create) {
	golem::XMLData("sim_delta_recv", val.simDeltaRecv, context, create);
	golem::XMLData("sim_delta_send", val.simDeltaSend, context, create);
}

void golem::XMLData(SingleCtrl::Desc &val, XMLContext* context, bool create) {
	XMLData((Controller::Desc&)val, context, create);

	golem::XMLDataIO(val, context->getContextFirst("io"), create);
	golem::XMLDataSim(val, context->getContextFirst("io"), create);

	try {
		XMLData(val.assertOffset, context->getContextFirst("assert_offset"), create);
	}
	catch (const MsgXMLParser& msg) {
		if (create) throw msg;
	}

	try {
		golem::XMLData("debug", val.debug, context, create);
	}
	catch (const MsgXMLParserAttributeNotFound&) {
	}
}

void golem::XMLData(SingleCtrl::CoordMap::Seq::value_type &val, XMLContext* context, bool create) {
	XMLData("src", val.src, context, create);
	XMLData("dst", val.dst, context, create);
	XMLData("gain", val.gain, context, create);
	XMLData("offset", val.offset, context, create);
}

//------------------------------------------------------------------------------
