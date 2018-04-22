/** @file GolemInteropStreamSocket.h
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
#ifndef _GOLEM_INTEROP_INTEROP_STREAM_SOCKET_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_INTEROP_STREAM_SOCKET_H_

//------------------------------------------------------------------------------

#include "GolemInteropStream.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio/deadline_timer.hpp>

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/*************************************************************************
	*
	* Client-server communication
	*
	**************************************************************************/

	class Server;
	class Client;

	/** Synchronous socket interface */
	class StreamSocket : public Stream {
	public:
		friend class Server;
		friend class Client;

		/** Read from stream */
		virtual void read(size_t size, void* data) {
			boost::system::error_code error;
			boost::asio::read(
				socket,
				boost::asio::buffer(data, size),
				boost::asio::transfer_all(),
				//boost::asio::transfer_exactly(size),
				error
			);
			if (error)
				throw std::runtime_error("StreamSocket::read(): unable to receive data: " + error.message());
		}
		/** Write to stream */
		virtual void write(size_t size, const void* data) {
			boost::system::error_code error;
			boost::asio::write(
				socket,
				boost::asio::buffer(data, size),
				boost::asio::transfer_all(),
				error
			);
			if (error)
				throw std::runtime_error("StreamSocket::write(): unable to send data: " + error.message());
		}

		/** Get socket */
		const boost::asio::ip::tcp::socket& getSocket() const {
			return socket;
		}

		/** Closes connection */
		~StreamSocket() {}

	protected:
		StreamSocket(boost::asio::ip::tcp::socket& socket) : socket(std::move(socket)) {}

		boost::asio::ip::tcp::socket socket;
	};

	/** Asynchronous socket interface */
	class StreamAsyncSocket : public Stream {
	public:
		friend class Server;
		friend class Client;

		/** Read from stream */
		virtual void read(size_t size, void* data) {
			boost::asio::async_read(
				socket,
				boost::asio::buffer(data, size),
				boost::asio::transfer_all(),
				//boost::asio::transfer_exactly(size),
				boost::bind(
					&StreamAsyncSocket::readHandler,
					boost::asio::placeholders::error
				)
			);

			while (socket.get_io_service().run_one());
			socket.get_io_service().reset();
		}
		/** Write to stream */
		virtual void write(size_t size, const void* data) {
			boost::asio::async_write(
				socket,
				boost::asio::buffer(data, size),
				boost::asio::transfer_all(),
				boost::bind(
					&StreamAsyncSocket::writeHandler,
					boost::asio::placeholders::error
				)
			);

			while (socket.get_io_service().run_one());
			socket.get_io_service().reset();
		}

		/** Get socket */
		const boost::asio::ip::tcp::socket& getSocket() const {
			return socket;
		}

		/** Closes connection */
		~StreamAsyncSocket() {}

	protected:
		static void readHandler(const boost::system::error_code& error) {
			if (error)
				throw std::runtime_error("StreamAsyncSocket::readHandler(): unable to send data: " + error.message());
		}
		static void writeHandler(const boost::system::error_code& error) {
			if (error)
				throw std::runtime_error("StreamAsyncSocket::writeHandler(): unable to send data: " + error.message());
		}

		StreamAsyncSocket(boost::asio::ip::tcp::socket& socket) : socket(std::move(socket)) {}

		boost::asio::ip::tcp::socket socket;
	};

	/** TCP/IP server */
	class Server {
	public:
		/** Create server, listen on specified port */
		Server(const unsigned short port) : acceptor(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)) {}

		/** Accept incoming connection */
		template <typename _Stream> typename _Stream::Ptr accept() {
			boost::asio::ip::tcp::socket socket(io_service);
			acceptor.accept(socket);
			return typename _Stream::Ptr(new _Stream(socket));
		}

		/** acceptor */
		boost::asio::ip::tcp::acceptor& getAcceptor() {
			return acceptor;
		}
		/** acceptor */
		const boost::asio::ip::tcp::acceptor& getAcceptor() const {
			return acceptor;
		}

	protected:
		boost::asio::io_service io_service;
		boost::asio::ip::tcp::acceptor acceptor;
	};

	/** TCP/IP client */
	class Client {
	public:
		/** Create client */
		Client() {}

		/** Initialise connection */
		template <typename _Stream> typename _Stream::Ptr connect(const std::string& host, const unsigned short port/*, unsigned timeoutMs = 10000*/) {
			boost::asio::ip::tcp::socket socket(io_service);
			boost::asio::ip::tcp::resolver resolver(io_service);
			boost::asio::ip::tcp::resolver::query query(boost::asio::ip::tcp::v4(), host, std::to_string(port));
			
			boost::system::error_code error;
			
			// IPv4 first
			std::vector<boost::asio::ip::tcp::resolver::iterator> endpoints;
			for (boost::asio::ip::tcp::resolver::iterator endpoint = resolver.resolve(query, error), end; endpoint != end; ++endpoint)
				endpoints.insert(endpoint->endpoint().address().is_v4() ? endpoints.begin() : endpoints.end(), endpoint);

			// try all endpoints
			for (std::vector<boost::asio::ip::tcp::resolver::iterator>::const_iterator endpoint = endpoints.begin(); endpoint != endpoints.end(); ++endpoint) {
				//boost::asio::deadline_timer timer(io_service, boost::posix_time::milliseconds(timeoutMs));

				socket.connect(**endpoint, error);
				//socket.async_connect(**endpoint, boost::bind(&Client::connectHandler, boost::asio::placeholders::error, &timer, &error));
				//timer.async_wait(boost::bind(&Client::timeoutHandler, boost::asio::placeholders::error, &socket, &error));
				
				//io_service.poll_one();
				//io_service.reset();

				if (!error)
					return typename _Stream::Ptr(new _Stream(socket));

				socket.close();
			}
			
			throw boost::system::system_error(error);
		}

	protected:
		boost::asio::io_service io_service;

		//static void connectHandler(const boost::system::error_code& errorRes, boost::asio::deadline_timer* timer, boost::system::error_code* error) {
		//	timer->cancel();
		//	*error = errorRes;
		//}
		//static void timeoutHandler(const boost::system::error_code& errorRes, boost::asio::ip::tcp::socket* socket, boost::system::error_code* error) {
		//	if (errorRes != boost::asio::error::operation_aborted) {
		//		socket->close();
		//		*error = errorRes;
		//	}
		//}
	};

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_INTEROP_STREAM_SOCKET_H_