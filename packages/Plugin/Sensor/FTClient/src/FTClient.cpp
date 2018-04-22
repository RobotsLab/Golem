/** @file FTClient.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sensor/FTClient/FTClient.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new FTClient::Desc();
}

//------------------------------------------------------------------------------

void golem::FTClient::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	FT::Desc::load(context, xmlcontext);

	golem::XMLData("host", host, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("port", port, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("record", record, const_cast<golem::XMLContext*>(xmlcontext), false);
}

//------------------------------------------------------------------------------

golem::FTClient::FTClient(golem::Context& context) : FT(context) {
}

void golem::FTClient::create(const Desc& desc) {
	FT::create(desc);

	try {
		record = desc.record;

		timer.reset(new SMTimer(&context.getTimer()));
		messageStream.reset(new SMMessageStream(*context.getMessageStream()));

		client.reset(new golem::SMClient(desc.host, desc.port, *timer, messageStream.get()));
		client->syncTimers();
		client->start();
		id = 0;
	}
	catch (const std::exception& ex) {
		throw Message(Message::LEVEL_CRIT, "FTClient::create(): F/T sensor not available (%s)", ex.what());
	}
}

void golem::FTClient::readSensor(golem::Twist& wrench, golem::SecTmReal& timeStamp) {
	golem::SM::Header header(MAX_RECORDS*sizeof(Twist), id);
	golem::Twist buff[MAX_RECORDS];

	if (!client->read(header, buff, boost::posix_time::seconds(0)))
		throw Message(Message::LEVEL_ERROR, "FTClient::readSensor(): Unable to read");
	
	if (header.size / sizeof(Twist) <= record)
		throw Message(Message::LEVEL_ERROR, "FTClient::readSensor(): Record %u not in range <0, %u), received size %u", record, header.size / sizeof(Twist), header.size);
	
	wrench = buff[record];
	timeStamp = header.time;

	//static int j = 0;
	//if (j++ % 10 == 0) {
	//	context.write("%s: (% 8.4f, % 8.4f, % 8.4f) (% 8.4f, % 8.4f, % 8.4f)\n", getID().c_str(), wrench.v.x, wrench.v.y, wrench.v.z, wrench.w.x, wrench.w.y, wrench.w.z);
	//}
}

//------------------------------------------------------------------------------
