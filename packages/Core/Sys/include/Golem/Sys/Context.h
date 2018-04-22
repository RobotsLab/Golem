/** @file Context.h
 * 
 * Context class is a collection of application helper objects.
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
#ifndef _GOLEM_SYS_CONTEXT_H_
#define _GOLEM_SYS_CONTEXT_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Defs.h>
#include <Golem/Math/Rand.h>
#include <Golem/Sys/Message.h>
#include <Golem/Sys/Library.h>
#include <Golem/Sys/Parallels.h>
#include <Golem/Sys/MessageStream.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgContext, Message)
MESSAGE_DEF(MsgContextInvalidDesc, MsgContext)

//------------------------------------------------------------------------------

/** Context object is a collection of shared object */
class Context {
public:
	typedef shared_ptr<Context> Ptr;
	friend class Desc;

	/** Context description */
	class Desc {
	public:
		/** Random generator seed */
		RandSeed randSeed;
		/** Message Stream */
		MessageStream::Desc::Ptr messageStreamDesc;
		/** Parallels thread joint time out */
		MSecTmU32 threadTimeOut;
		/** Number of threads in Parallels */
		U32 threadParallels;

		/** Constructs context description. */
		Desc() {
			setToDefault();
		}
		
		/** Virtual destructor */
		virtual ~Desc() {}
		
		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			messageStreamDesc.reset(new BufferedStream::Desc);
			threadTimeOut = 5000;
			threadParallels = 0; // inactive by default
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (messageStreamDesc == NULL || !messageStreamDesc->isValid())
				return false;
			if (threadTimeOut <= 0)
				return false;

			return true;
		}
		
		/** Creates object from the description. */
		CREATE_FROM_OBJECT_DESC_0(Context, Context::Ptr)
	};
	
protected:
	/** Precision/performance timer */
	PerfTimer timer;
	/** Random generator seed */
	RandSeed randSeed;
	/** Message stream */
	MessageStream::Ptr messages;
	/** Parallels thread joint time out */
	MSecTmU32 threadTimeOut;
	/** Parallels */
	shared_ptr<Parallels> parallels;
	/** Library map */
	Handle::Map libraryMap;
	/** Library list */
	Handle::List libraryList;

	/** Creates context from description */
	void create(const Desc &desc);

	/** Default constructror */
	Context();

public:
	/** Virtual destructor */
	virtual ~Context();
	
	/** Initialise context in the current module */
	void initModule();

	/** Returns pointer to timer */
	inline const PerfTimer& getTimer() const {
		return timer;
	}

	/** Returns reference to randseed */
	inline const RandSeed& getRandSeed() const {
		return randSeed;
	}

	/** Returns pointer to message stream */
	inline const MessageStream* getMessageStream() const {
		return messages.get();
	}
	inline MessageStream* getMessageStream() {
		return messages.get();
	}

	/** Message stream direct access */
	#define CONTEXT_MESSAGE_STREAM(NAME, TIME, THREAD, LEVEL, CODE)\
		void NAME(const char* format, ...) const {\
			va_list argptr;\
			va_start(argptr, format);\
			messages->write(TIME, THREAD, LEVEL, Message::CODE_UNDEF, format, argptr);\
			va_end(argptr);\
		}

	CONTEXT_MESSAGE_STREAM(write, Message::TIME_UNDEF, Message::THREAD_UNDEF, Message::LEVEL_UNDEF, Message::CODE_UNDEF)
	CONTEXT_MESSAGE_STREAM(verbose, Message::getCurrentTime(), Message::getCurrentThread(), Message::LEVEL_VERBOSE, Message::CODE_UNDEF)
	CONTEXT_MESSAGE_STREAM(debug, Message::getCurrentTime(), Message::getCurrentThread(), Message::LEVEL_DEBUG, Message::CODE_UNDEF)
	CONTEXT_MESSAGE_STREAM(info, Message::getCurrentTime(), Message::getCurrentThread(), Message::LEVEL_INFO, Message::CODE_UNDEF)
	CONTEXT_MESSAGE_STREAM(notice, Message::getCurrentTime(), Message::getCurrentThread(), Message::LEVEL_NOTICE, Message::CODE_UNDEF)
	CONTEXT_MESSAGE_STREAM(warning, Message::getCurrentTime(), Message::getCurrentThread(), Message::LEVEL_WARNING, Message::CODE_UNDEF)
	CONTEXT_MESSAGE_STREAM(error, Message::getCurrentTime(), Message::getCurrentThread(), Message::LEVEL_ERROR, Message::CODE_UNDEF)
	CONTEXT_MESSAGE_STREAM(crit, Message::getCurrentTime(), Message::getCurrentThread(), Message::LEVEL_CRIT, Message::CODE_UNDEF)

	#undef CONTEXT_MESSAGE_STREAM

	/** Write message */
	inline void write(const Message& msg, const char* str = "\n") {
		messages->write(msg.time(), msg.thread(), msg.level(), msg.code(), "%s%s", msg.msg(), str);
	}

	/** Returns pointer to parallels */
	inline const Parallels* getParallels() const {
		return parallels.get();
	}
	inline Parallels* getParallels() {
		return parallels.get();
	}

	/** Get library */
	Handle getLibrary(const std::string& path);
	/** Release libraries */
	void releaseLibraries();
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_SYS_CONTEXT_H_*/
