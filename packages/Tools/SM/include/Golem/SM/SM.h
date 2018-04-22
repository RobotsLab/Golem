/** @file sm.h
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
#ifndef _GOLEM_SM_SM_H_
#define _GOLEM_SM_SM_H_

#include <Golem/Defs/System.h>
#ifdef WIN32
	#pragma warning (push)
	#pragma warning (disable : 4996 4251 4217)
#endif // WIN32
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#ifdef WIN32
	#pragma warning (pop)
#endif
#include <vector>
#include <stdio.h>
#include <stdarg.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class SM {
public:
	class MessageStream {
	public:
		virtual ~MessageStream() {}
	
		#define GOLEM_DEVICE_MESSAGE_STREAM(NAME, FILE)\
			virtual void NAME(const char* format, va_list argptr) const {\
				vfprintf(FILE, format, argptr);\
			}\
			static void NAME(MessageStream* msgStr, const char* format, ...) {\
				if (msgStr) {\
					va_list argptr;\
					va_start(argptr, format);\
					msgStr->NAME(format, argptr);\
					va_end(argptr);\
				}\
			}

		GOLEM_DEVICE_MESSAGE_STREAM(debug, stdout)
		GOLEM_DEVICE_MESSAGE_STREAM(info, stdout)
		GOLEM_DEVICE_MESSAGE_STREAM(error, stderr)

		#undef GOLEM_DEVICE_MESSAGE_STREAM
	};

	class Timer {
	public:
		virtual ~Timer() {}
		virtual double elapsed() const = 0;
		virtual void sleep(double sec) const = 0;
	};

	class Header {
	public:
		unsigned size;
		unsigned id;
		double time;

		Header(unsigned size = 0, unsigned id = 0, double time = 0.0) : size(size), id(id), time(time) {}
	};

	const Timer& timer;
	MessageStream* const msgStr;
	
	SM(const Timer& timer, MessageStream* msgStr = NULL);

protected:
	class Command {
	public:
		enum Type {
			TYPE_NONE,
			TYPE_EXIT,
			TYPE_DATA_START,
			TYPE_DATA_STOP,
			TYPE_DATA_ASYNC,
		};
		static const char* Name [];

		int type;

		Command(int type = TYPE_NONE);
	};

	class Data {
	public:
		Data();

		void resizeData(unsigned size);
		void reserveData(unsigned capacity);

		inline unsigned size() const {
			return (unsigned)buffer.size();
		}
		
		inline Header* getHeader() {
			return (Header*)(&buffer[0]);
		}
		inline const Header* getHeader() const {
			return (const Header*)(&buffer[0]);
		}

		inline void* getData() {
			return &buffer[sizeof(Header)];
		}
		inline const void* getData() const {
			return &buffer[sizeof(Header)];
		}
		
		inline void* get() {
			return &buffer[0];
		}
		inline const void* get() const {
			return &buffer[0];
		}

	private:
		// TODO: use malloc + free + memcpy for fast memory transfers
		typedef std::vector<char> Buffer;

		Buffer buffer;
	};

	class Sync {
	public:
		unsigned id;
		double time;

		Sync(unsigned id = 0, double time = 0.0) : id(id), time(time) {}
	};

	inline void read(Data& data) {
		boost::lock_guard<boost::mutex> lock(smmutex);
		data = smdata;
	}
	inline void read(Header& header, void* data) {
		boost::lock_guard<boost::mutex> lock(smmutex);
		get(header, data);
	}

	inline void write(const Data& data) {
		boost::lock_guard<boost::mutex> lock(smmutex);
		smdata = data;
		smcondition.notify_all();
	}
	inline void write(unsigned size, const void* data) {
		boost::lock_guard<boost::mutex> lock(smmutex);
		set(size, data);
		smcondition.notify_all();
	}

	inline void wakeup() {
		smcondition.notify_all();
	}

	template<typename duration_type> inline bool wait(unsigned id, const duration_type& wait_duration) {
		boost::unique_lock<boost::mutex> lock(smmutex);
		return smcondition.timed_wait(lock, wait_duration) || id != smdata.getHeader()->id;
	}

private:
	Data smdata;
	boost::mutex smmutex;
	boost::condition_variable smcondition;

	void get(Header& header, void* data) const;
	void set(const Header& header, const void* data);
	void set(unsigned size, const void* data);
};

//------------------------------------------------------------------------------

class SMServer : public SM {
public:
	class Endpoint {
	public:
		typedef std::vector<Endpoint> Seq;

		std::string address;
		unsigned short port;

		Endpoint(const std::string& address, unsigned short port) : address(address), port(port) {}
	};

	class Handler {
	public:
		virtual ~Handler() {}
		virtual void set(const Endpoint& endpoint, const Header& header, const void* data) = 0;
	};
	
	SMServer(unsigned short port, const Timer& timer, MessageStream* msgStr = NULL, Handler* handler = NULL, size_t clients = 10, unsigned timeout = 1000);
	~SMServer();

	inline void write(unsigned size, const void* data)  {
		SM::write(size, data);
	}

	Endpoint::Seq remotes();

protected:
	class Connection {
	public:
		typedef boost::shared_ptr<Connection> Ptr;
		typedef std::map<const Connection*, Ptr> List;

		SMServer* server;
		boost::asio::ip::tcp::socket tcpsocket;
		Command command;
		Data inpbuffer, outbuffer;
		
		boost::shared_ptr<boost::thread> thread;
		bool exit, transfer, ready;

		boost::mutex mutex;

		static Ptr create(SMServer* server);
		
		Connection(SMServer* server);
		~Connection();

		void start();
		void run();
		void dataHandler(const boost::system::error_code& error, size_t bytes_transferred);

		boost::asio::ip::tcp::socket& socket() {
			return tcpsocket;
		}

		Endpoint remote() const {
			return Endpoint(tcpsocket.remote_endpoint().address().to_string(), tcpsocket.remote_endpoint().port());
		}
	};

	const unsigned short port;
	const size_t clients;
	const unsigned timeout;
	Handler* handler;
	boost::asio::io_service io_service;
	boost::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor;
	boost::shared_ptr<boost::asio::ip::udp::socket> udpsocket;
	boost::shared_ptr<boost::thread> thread;

	Connection::List waitList, connectionList;
	boost::mutex listMutex;
	boost::condition_variable listCondition;

	void run();

	void startConnection();
	void deleteConnection(Connection* connection);
	void acceptConnectionHandler(Connection* connection, const boost::system::error_code& error);
	void startConnectionCommand(Connection* connection);
	void connectionCommandHandler(Connection* connection, const boost::system::error_code& error, size_t bytes_transferred);
	void connectionHeaderHandler(Connection* connection, const boost::system::error_code& error, size_t bytes_transferred);
	void connectionDataHandler(Connection* connection, const boost::system::error_code& error, size_t bytes_transferred);
	
	void startSync() const;
	void acceptSyncHandler(boost::shared_ptr<boost::asio::ip::udp::endpoint> syncendpoint, boost::shared_ptr<Sync> sync, const boost::system::error_code& error, size_t bytes_transferred) const;
	void syncHandler(boost::shared_ptr<Sync> sync, const boost::system::error_code& error, size_t bytes_transferred) const;
};

//------------------------------------------------------------------------------

class SMClient : public SM {
public:
	SMClient(const std::string& host, unsigned short port, const Timer& timer, MessageStream* msgStr = NULL);
	~SMClient();

	void syncTimers(unsigned syncTries = 10, double syncWait = 0.1);

	void start();
	void stop();

	inline void read(Header& header, void* data) {
		if (exit)
			throw std::runtime_error("SMClient::read(): disconnected");
		SM::read(header, data);
	}
	template<typename duration_type> inline bool read(Header& header, void* data, const duration_type& wait_duration) {
		const bool success = SM::wait(header.id, wait_duration);
		if (exit)
			throw std::runtime_error("SMClient::read(): disconnected");
		if (success)
			SM::read(header, data);
		return success;
	}
	void write(unsigned size, const void* data);

	inline double toServerTime(double t) const {
		return t + timersOffset;
	}
	inline double toClientTime(double t) const {
		return t - timersOffset;
	}

protected:
	boost::asio::io_service io_service;
	boost::shared_ptr<boost::asio::ip::tcp::socket> tcpsocket;
	boost::shared_ptr<boost::thread> thread;
	bool exit;
	double timersOffset;
	unsigned id;

	Command makeCommand(Command::Type type) const;
	void sendCommand(Command::Type type);
	void commandHandler(const boost::system::error_code& error, size_t bytes_transferred);
	void dataHandler(const boost::system::error_code& error, size_t bytes_transferred, size_t bytes);
	void recvData(Data& data);

	void run();
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_SM_SM_H_*/
