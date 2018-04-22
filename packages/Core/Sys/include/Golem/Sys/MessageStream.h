/** @file MessageStream.h
 * 
 * Message Stream.
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
#ifndef _GOLEM_SYS_MESSAGESTREAM_H_
#define _GOLEM_SYS_MESSAGESTREAM_H_

//------------------------------------------------------------------------------

#include <Golem/Sys/Message.h>
#include <Golem/Math/Queue.h>
#include <Golem/Sys/Thread.h>
#include <Golem/Defs/Defs.h>
#include <iostream>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgMessageStream, Message)
MESSAGE_DEF(MsgMessageStreamInvalidDesc, MsgMessageStream)
MESSAGE_DEF(MsgMessageStreamThreadLaunch, MsgMessageStream)

//------------------------------------------------------------------------------

/** Message filter */
class MessageFilter {
public:
	typedef shared_ptr<MessageFilter> Ptr;

	virtual ~MessageFilter() {}

	/** Returns true if accepted, false otherwise */
	virtual bool operator () (SecTmReal time, U32 thread, Message::Level level, Message::Code code) const = 0;
};

/** Accepts messages with level equal or higher than specified */
class LevelMessageFilter: public MessageFilter {
public:
	LevelMessageFilter(Message::Level level = Message::LEVEL_INFO) : _level(level) {}
	
	virtual bool operator () (SecTmReal time, U32 thread, Message::Level level, Message::Code code) const {
		return level >= _level || level == Message::LEVEL_UNDEF;
	}

private:
	const Message::Level _level;
};

//------------------------------------------------------------------------------

/** MessageCallback */
class MessageCallback {
public:
	typedef shared_ptr<MessageCallback> Ptr;

	virtual ~MessageCallback() {}
	
	/** MessageStream::Callback: write */
	virtual void write(const Message& message) = 0;
};

/** C++ stream MessageCallback */
class StreamMessageCallback : public MessageCallback {
public:
	/** MessageCallback description */
	class Desc {
	public:
		/** File */
		FILE* file;

		/** Enable formatting */
		bool enableFormat;

		/** Escape sequence reset */
		std::string escSeqReset;
		/** Escape sequence */
		std::string escSeqLevel[Message::LEVEL_CRIT + 1];

		/** Constructs Logger description. */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			enableFormat = false;

			file = stdout;

			//	ANSI/VT100 - 4-bit Colours
			//
			//  Code        Effect                           Note                                   
			//
			//  0           Reset / Normal                   all attributes off                                                      
			//  1           Bold or increased intensity                                                                              
			//  2           Faint (decreased intensity)      Not widely supported.                                                   
			//  3           Italic                           Not widely supported. Sometimes treated as inverse.                     
			//  4           Underline                                                                                                
			//  5           Slow Blink                       less than 150 per minute                                                
			//  6           Rapid Blink                      MS-DOS ANSI.SYS; 150+ per minute; not widely supported                  
			//  7           [[reverse video]]                swap foreground and background colors                                   
			//  8           Conceal                          Not widely supported.                                                   
			//  9           Crossed-out                      Characters legible, but marked for deletion.  Not widely supported.     
			//  10          Primary(default) font                                                                                    
			//  11–19       Alternate font                   Select alternate font `n-10`                                            
			//  20          Fraktur                          hardly ever supported                                                   
			//  21          Bold off or Double Underline     Bold off not widely supported; double underline hardly ever supported.  
			//  22          Normal color or intensity        Neither bold nor faint                                                  
			//  23          Not italic, not Fraktur                                                                                  
			//  24          Underline off                    Not singly or doubly underlined                                         
			//  25          Blink off                                                                                                
			//  27          Inverse off                                                                                              
			//  28          Reveal                           conceal off                                                             
			//  29          Not crossed out                                                                                          
			//  30–37       Set foreground color             See color table below                                                   
			//  38          Set foreground color             Next arguments are `5;n` or `2;r;g;b`, see below                        
			//  39          Default foreground color         implementation defined (according to standard)                          
			//  40–47       Set background color             See color table below                                                   
			//  48          Set background color             Next arguments are `5;n` or `2;r;g;b`, see below                        
			//  49          Default background color         implementation defined (according to standard)                          
			//  51          Framed                                                                                                   
			//  52          Encircled                                                                                                
			//  53          Overlined                                                                                                
			//  54          Not framed or encircled                                                                                  
			//  55          Not overlined                                                                                            
			//  60          ideogram underline               hardly ever supported                                                   
			//  61          ideogram double underline        hardly ever supported                                                   
			//  62          ideogram overline                hardly ever supported                                                   
			//  63          ideogram double overline         hardly ever supported                                                   
			//  64          ideogram stress marking          hardly ever supported                                                   
			//  65          ideogram attributes off          reset the effects of all of 60-64                                       
			//  90–97       Set bright foreground color      aixterm (not in standard)                                               
			//  100–107     Set bright background color      aixterm (not in standard)                                               
			//
			//	0 	Black/Gray
			//	1 	Red
			//	2 	Green
			//	3 	Yellow
			//	4 	Blue
			//	5 	Magenta
			//	6 	Cyan
			//	7 	White/Gray

			escSeqReset = "\033[0m";

			escSeqLevel[0] = "\033[39m";	// LEVEL_UNDEF
			escSeqLevel[1] = "\033[90m";	// LEVEL_VERBOSE
			escSeqLevel[2] = "\033[37m";	// LEVEL_DEBUG
			escSeqLevel[3] = "\033[97m";	// LEVEL_INFO
			escSeqLevel[4] = "\033[93m";	// LEVEL_NOTICE
			escSeqLevel[5] = "\033[93m";	// LEVEL_WARNING
			escSeqLevel[6] = "\033[91m";	// LEVEL_ERROR
			escSeqLevel[7] = "\033[91m";	// LEVEL_CRIT
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (file == nullptr)
				return false;
			return true;
		}

	};

	/** StreamMessageCallback: construction */
	StreamMessageCallback(const Desc& desc = Desc(), std::ostream& ostr = std::cout);

	/** StreamMessageCallback: write */
	virtual void write(const Message& message);

private:
	Desc desc;
	std::ostream& ostr;
};

//------------------------------------------------------------------------------

/** Message stream with message filtering */
class MessageStream {
public:
	typedef shared_ptr<MessageStream> Ptr;

	/** Message logger description */
	class Desc {
	public:
		typedef shared_ptr<Desc> Ptr;
		
		/** Message filter, accept all messages if NULL */
		MessageFilter::Ptr messageFilter;
		/** MessageStream::Callback */
		MessageCallback::Ptr messageCallback;

		/** Constructs Logger description. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** Virtual destructor */
		virtual ~Desc() {}
		
		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			messageFilter.reset();
			messageCallback.reset();
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (messageCallback == nullptr)
				return false;
			return true;
		}
		
		/** Creates Logger from the description. */
		virtual MessageStream::Ptr create() const = 0;
	};

protected:
	/** Message filter */
	MessageFilter::Ptr filter;
	/** Message callback */
	MessageCallback::Ptr callback;

	/** Creates object from description */
	void create(const Desc &desc);

	/** Constructor */
	MessageStream();

public:
	/** Virtual destructor */
	virtual ~MessageStream() {}
	
	/** Writes given message to the stream bypassing filter */
	virtual void write(const Message& message) = 0;
	
	/** Constructs a message and writes to the stream */
	virtual void write(SecTmReal time, U32 thread, Message::Level level, Message::Code code, const char* format, va_list argptr);

	/** Constructs a message and writes to the stream */
	virtual void write(const char* format, ...);
	
	/** Constructs a message and writes to the stream */
	virtual void write(Message::Level level, const char* format, ...);
	
	/** Constructs a message and writes to the stream */
	virtual void write(Message::Level level, Message::Code code, const char* format, ...);

	/** Constructs a message and writes to the stream */
	virtual void write(SecTmReal time, U32 thread, Message::Level level, Message::Code code, const char* format, ...);
};

//------------------------------------------------------------------------------

/** Output buffered stream */
class BufferedStream : public MessageStream, public Runnable {
public:
	friend class Desc;

	/** Message logger description */
	class Desc : public MessageStream::Desc {
	public:
		/** Message queue size */
		U32 queueSize;
		/** Working thread priority */
		Thread::Priority threadPriority;
		/** Working thread time out */
		MSecTmU32 threadTimeOut;

		/** Constructs Logger description. */
		Desc() {
			Desc::setToDefault();
		}
		
		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			MessageStream::Desc::setToDefault();
			
			messageFilter.reset(new LevelMessageFilter());
			messageCallback.reset(new StreamMessageCallback());
			
			queueSize = 1000;
			threadPriority = Thread::NORMAL;
			threadTimeOut = 30000; // 30 sec
		}
		
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!MessageStream::Desc::isValid())
				return false;

			if (queueSize <= 0 || threadTimeOut <= 0)
				return false;
			
			return true;
		}

		/** Creates object from the description. */
		CREATE_FROM_OBJECT_DESC_0(BufferedStream, MessageStream::Ptr)
	};

protected:
	golem::queue<Message> messages;
	MSecTmU32 threadTimeOut;
	Thread thread;
	Event evQueue;
	CriticalSection csQueue;
	bool terminate;

	/** Reads stream */
	virtual void run();

	/** Creates object from description */
	void create(const Desc &desc);

public:
	/** Virtual destructor */
	virtual ~BufferedStream();

	/** Writes given message to the stream bypassing filter */
	virtual void write(const Message& message);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_SYS_MESSAGESTREAM_H_*/
