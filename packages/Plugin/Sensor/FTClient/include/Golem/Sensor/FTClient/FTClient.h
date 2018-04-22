/** @file FTClient.h
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_CORE_FTCLIENT_H_
#define _GOLEM_CORE_FTCLIENT_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/FT.h>
#include <Golem/SM/SMHelper.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** FT sensor network client */
class GOLEM_LIBRARY_DECLDIR FTClient : public FT {
public:
	/** Maximum number of F/T records */
	static const golem::U32 MAX_RECORDS = 12;

	/** FTSensor description */
	class GOLEM_LIBRARY_DECLDIR Desc : public FT::Desc {
	public:
		/** FT server host name */
		std::string host;
		/** FT server host port */
		unsigned short port;
		/** FT server record index */
		golem::U32 record;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			FT::Desc::setToDefault();

			host = "localhost";
			port = 26873;
			record = 0;
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			FT::Desc::assertValid(ac);

			Assert::valid(host.length() > 0, ac, "host: empty");
			Assert::valid(port > 0, ac, "port: invalid");
			Assert::valid(record < MAX_RECORDS, ac, "record: too large");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		GOLEM_CREATE_FROM_OBJECT_DESC1(FTClient, Sensor::Ptr, golem::Context&)
	};

	/** Read newest F&T, blocking call */
	virtual void readSensor(golem::Twist& wrench, golem::SecTmReal& timeStamp); // throws

protected:
	/** Client */
	boost::shared_ptr<golem::SMClient> client;
	/** Timer */
	boost::shared_ptr<golem::SMTimer> timer;
	/** Message stream */
	boost::shared_ptr<golem::SMMessageStream> messageStream;
	/** FT server record index */
	golem::U32 record;
	/** Packet id */
	unsigned id;

	/** Creates/initialises sensor */
	void create(const Desc& desc);
	/** Constructs sensor */
	FTClient(golem::Context& context);
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_CORE_FTCLIENT_H_*/
