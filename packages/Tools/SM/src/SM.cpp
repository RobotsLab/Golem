/** @file sm.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/SM/SM.h>
#include <map>
#include <boost/lexical_cast.hpp>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const char* golem::SM::Command::Name [] = {
	"none",
	"close connection",
	"data transfer start",
	"data transfer stop",
	"async data transfer",
};

SM::Command::Command(int type) : type(type) {
}

SM::Data::Data() {
	resizeData(0);
	getHeader()->id = 0;
	getHeader()->time = 0.0;
}

void SM::Data::resizeData(unsigned size) {
	buffer.resize(size + sizeof(Header));
	getHeader()->size = size;
}

void SM::Data::reserveData(unsigned capacity) {
	buffer.reserve(capacity + sizeof(Header));
	resizeData(0);
}

SM::SM(const Timer& timer, MessageStream* msgStr) : timer(timer), msgStr(msgStr) {
	smdata.resizeData(0);
}

void SM::get(Header& header, void* data) const {
	header.size = std::min(header.size, smdata.getHeader()->size);
	header.id = smdata.getHeader()->id;
	header.time = smdata.getHeader()->time;
	std::copy((const char*)smdata.getData(), (const char*)smdata.getData() + header.size, (char*)data);
}

void SM::set(const Header& header, const void* data) {
	smdata.resizeData(header.size);
	smdata.getHeader()->id = header.id;
	smdata.getHeader()->time = header.time;
	std::copy((const char*)data, (const char*)data + smdata.getHeader()->size, (char*)smdata.getData());
}

void SM::set(unsigned size, const void* data) {
	smdata.resizeData(size);
	smdata.getHeader()->id++;
	smdata.getHeader()->time = timer.elapsed();
	std::copy((const char*)data, (const char*)data + smdata.getHeader()->size, (char*)smdata.getData());
}

//------------------------------------------------------------------------------

SMServer::SMServer(unsigned short port, const Timer& timer, MessageStream* msgStr, Handler* handler, size_t clients, unsigned timeout) : SM(timer, msgStr), port(port), handler(handler), clients(clients), timeout(timeout) {
	acceptor.reset(new boost::asio::ip::tcp::acceptor(
		io_service,
		boost::asio::ip::tcp::endpoint(
			boost::asio::ip::tcp::v4(),
			port
		)
	));
	
	udpsocket.reset(new boost::asio::ip::udp::socket(
		io_service,
		boost::asio::ip::udp::endpoint(
			boost::asio::ip::udp::v4(),
			port
		)
	));

	startConnection();
	startSync();

	thread.reset(new boost::thread(&SMServer::run, this));
}

SMServer::~SMServer() {
	io_service.stop();
	if (thread)
		thread->join();
}

SMServer::Endpoint::Seq SMServer::remotes() {
	Endpoint::Seq endpoints;
	{
		boost::lock_guard<boost::mutex> lock(listMutex);
		for (Connection::List::const_iterator i = connectionList.begin(); i != connectionList.end(); ++i)
			endpoints.push_back(i->second->remote());
	}
	return endpoints;
}

void SMServer::run() {
	try {
		MessageStream::info(msgStr, "SMServer::run(): listening on port: %i\n", port);
		io_service.run();
	}
	catch (const std::exception& ex) {
		// report error and exit
		MessageStream::error(msgStr, "SMServer::run(): %s\n", ex.what());
	}
}

void SMServer::startConnection() {
	Connection::Ptr connection = Connection::create(this);
	
	{
		boost::lock_guard<boost::mutex> lock(listMutex);
		waitList[connection.get()] = connection;
	}

	acceptor->async_accept(
		connection->socket(),
		boost::bind(
			&SMServer::acceptConnectionHandler,
			this,
			connection.get(),
			boost::asio::placeholders::error
		)
	);
}

void SMServer::deleteConnection(Connection* connection) {
	boost::lock_guard<boost::mutex> lock(listMutex);
	connectionList.erase(connectionList.find(connection));
}

void SMServer::acceptConnectionHandler(Connection* connection, const boost::system::error_code& error) {
	if (error)
		return;

	const Endpoint endpoint = connection->remote();
	MessageStream::info(msgStr, "SMServer::acceptConnectionHandler(): host: %s, port: %i\n", endpoint.address.c_str(), endpoint.port);

	{
		boost::lock_guard<boost::mutex> lock(listMutex);
		Connection::List::iterator ptr = waitList.find(connection);
		if (ptr != waitList.end()) {
			connectionList[ptr->first] = ptr->second;
			waitList.erase(ptr);
		}
	}

	startConnectionCommand(connection);

	startConnection();
}

void SMServer::startConnectionCommand(Connection* connection) {
	connection->start();

	boost::lock_guard<boost::mutex> lock(connection->mutex);

	//MessageStream::debug(msgStr, "SMServer::startConnectionCommand(): async_read(): bytes=%u\n", sizeof(Command));

	boost::asio::async_read(
		connection->tcpsocket,
		boost::asio::buffer(&connection->command, sizeof(Command)),
#if (BOOST_VERSION >= 104700)
		boost::asio::transfer_exactly(sizeof(Command)),
#else
		boost::asio::transfer_at_least(sizeof(Command)),
#endif
		boost::bind(
			&SMServer::connectionCommandHandler,
			this,
			connection,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred
		)
	);
}

void SMServer::connectionCommandHandler(Connection* connection, const boost::system::error_code& error, size_t bytes_transferred) {
	if (connection->exit) {
		deleteConnection(connection);
		return;
	}

	if (error || bytes_transferred != sizeof(Command)) {
		MessageStream::error(msgStr, "SMServer::connectionCommandHandler(): error=%s, bytes=%u/%u\n", error.message().c_str(), (unsigned)bytes_transferred, (unsigned)sizeof(Command));
		connection->command.type = Command::TYPE_EXIT;
	}

	//MessageStream::debug(msgStr, "SMServer::connectionCommandHandler(): %s\n", Command::Name[connection->command.type]);

	switch (connection->command.type) {
	case Command::TYPE_NONE:
		break;
	case Command::TYPE_DATA_START:
		connection->transfer = true;
		break;
	case Command::TYPE_DATA_STOP:
		connection->transfer = false;
		break;
	case Command::TYPE_DATA_ASYNC:
		if (handler) {
			boost::lock_guard<boost::mutex> lock(connection->mutex);

			//boost::asio::async_read(
			//	connection->tcpsocket,
			//	boost::asio::buffer(connection->inpbuffer.get(), sizeof(Header)),
			//	boost::asio::transfer_exactly(sizeof(Header)),
			//	boost::bind(
			//		&SMServer::connectionHeaderHandler,
			//		this,
			//		connection,
			//		boost::asio::placeholders::error,
			//		boost::asio::placeholders::bytes_transferred
			//	)
			//);

			boost::asio::read(
				connection->tcpsocket,
				boost::asio::buffer(connection->inpbuffer.get(), sizeof(Header)),
#if (BOOST_VERSION >= 104700)
				boost::asio::transfer_exactly(sizeof(Header))
#else
				boost::asio::transfer_at_least(sizeof(Header))
#endif
			);

			//MessageStream::debug(msgStr, "SMServer::connectionCommandHandler(): async_read(): bytes=%u\n", connection->inpbuffer.getHeader()->size);

			connection->inpbuffer.resizeData(connection->inpbuffer.getHeader()->size);
			boost::asio::async_read(
				connection->tcpsocket,
				boost::asio::buffer(connection->inpbuffer.getData(), connection->inpbuffer.getHeader()->size),
				boost::asio::transfer_at_least(connection->inpbuffer.getHeader()->size),
				boost::bind(
					&SMServer::connectionDataHandler,
					this,
					connection,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred
				)
			);
			
			return;
		}
		else {
			MessageStream::error(msgStr, "SMServer::connectionCommandHandler(): data handler not set\n");
			break;
		}
	default:
		deleteConnection(connection);
		return;
	}

	startConnectionCommand(connection);
}

void SMServer::connectionHeaderHandler(Connection* connection, const boost::system::error_code& error, size_t bytes_transferred) {
	if (error || bytes_transferred != sizeof(Header)) {
		MessageStream::error(msgStr, "SMServer::connectionHeaderHandler(): error=%s, bytes=%u/%u\n", error.message().c_str(), (unsigned)bytes_transferred, (unsigned)sizeof(Header));
		connection->command.type = Command::TYPE_EXIT;
		return;
	}

	boost::lock_guard<boost::mutex> lock(connection->mutex);

	connection->inpbuffer.resizeData(connection->inpbuffer.getHeader()->size);

	boost::asio::async_read(
		connection->tcpsocket,
		boost::asio::buffer(connection->inpbuffer.getData(), connection->inpbuffer.getHeader()->size),
		boost::asio::transfer_at_least(connection->inpbuffer.getHeader()->size),
		boost::bind(
			&SMServer::connectionDataHandler,
			this,
			connection,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred
		)
	);
}

void SMServer::connectionDataHandler(Connection* connection, const boost::system::error_code& error, size_t bytes_transferred) {
	//MessageStream::debug(msgStr, "SMServer::connectionDataHandler(): bytes=%u/%u\n", (unsigned)bytes_transferred, (unsigned)connection->inpbuffer.getHeader()->size);
	
	if (error || bytes_transferred != connection->inpbuffer.getHeader()->size) {
		MessageStream::error(msgStr, "SMServer::connectionDataHandler(): error=%s, bytes=%u/%u\n", error.message().c_str(), (unsigned)bytes_transferred, (unsigned)connection->inpbuffer.getHeader()->size);
		connection->command.type = Command::TYPE_EXIT;
		return;
	}

	if (handler)
		handler->set(connection->remote(), *connection->inpbuffer.getHeader(), connection->inpbuffer.getData());

	startConnectionCommand(connection);
}

void SMServer::startSync() const {
	boost::shared_ptr<boost::asio::ip::udp::endpoint> syncendpoint(new boost::asio::ip::udp::endpoint);
	boost::shared_ptr<Sync> sync(new Sync);

	udpsocket->async_receive_from(
		boost::asio::buffer(sync.get(), sizeof(Sync)),
		*syncendpoint,
		boost::bind(
			&SMServer::acceptSyncHandler,
			this,
			syncendpoint,
			sync,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred
		)
	);
}

void SMServer::acceptSyncHandler(boost::shared_ptr<boost::asio::ip::udp::endpoint> syncendpoint, boost::shared_ptr<Sync> syncin, const boost::system::error_code& error, size_t bytes_transferred) const {
	//MessageStream::debug(msgStr, "SMServer::acceptSyncHandler(): host: %s, port: %i\n", syncendpoint->address().to_string().c_str(), syncendpoint->port());
	
	if (!error || error == boost::asio::error::message_size) {
		boost::shared_ptr<Sync> syncout(new Sync(syncin->id, timer.elapsed()));

		udpsocket->async_send_to(
			boost::asio::buffer(syncout.get(), sizeof(Sync)),
			*syncendpoint,
			boost::bind(
				&SMServer::syncHandler,
				this,
				syncout,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred
			)
		);
	}

	startSync();
}

void SMServer::syncHandler(boost::shared_ptr<Sync> sync, const boost::system::error_code& error, size_t bytes_transferred) const {
	if (error || bytes_transferred != sizeof(Sync)) {
		MessageStream::error(msgStr, "SMServer::syncHandler(): error=%s, bytes=%u/%u\n", error.message().c_str(), (unsigned)bytes_transferred, (unsigned)sizeof(Sync));
	}
}

//------------------------------------------------------------------------------

SMServer::Connection::Connection(SMServer* server) :
	server(server),
	tcpsocket(server->acceptor->get_io_service())
{
	exit = false;
	ready = true;
	transfer = false;
}

SMServer::Connection::~Connection() {
	exit = true;
	ready = false;
	
	try {
		tcpsocket.close();
	}
	catch (const std::exception&) {
	}

	if (thread)
		thread->join();
}

SMServer::Connection::Ptr SMServer::Connection::create(SMServer* server) {
	return Ptr(new Connection(server));
}

void SMServer::Connection::start() {
	if (!thread)
		thread.reset(new boost::thread(&Connection::run, this));
}

void SMServer::Connection::run() {
	while (!exit) {
		try {
			if (!server->wait(outbuffer.getHeader()->id, boost::posix_time::milliseconds(server->timeout)) || !ready || !transfer)
				continue;
			server->read(outbuffer);
			ready = false;

			boost::lock_guard<boost::mutex> lock(mutex);

			boost::asio::async_write(
				tcpsocket,
				boost::asio::buffer(outbuffer.get(), outbuffer.size()),
				boost::asio::transfer_at_least(outbuffer.size()),
				boost::bind(
					&Connection::dataHandler,
					this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred
				)
			);
		}
		catch (const std::exception& ex) {
			// report error and stop
			MessageStream::error(server->msgStr, "SMServer::Connection::run(): %s\n", ex.what());
			exit = true;
		}
	}
}

void SMServer::Connection::dataHandler(const boost::system::error_code& error, size_t bytes_transferred) {
	if (exit)
		return;

	const size_t size = outbuffer.size();
	
	if (error || bytes_transferred != size) {
		MessageStream::error(server->msgStr, "SMServer::Connection::dataHandler(): error=%s, bytes=%u/%u\n", error.message().c_str(), (unsigned)bytes_transferred, (unsigned)size);
		exit = true;
	}
	else {
		ready = true;
	}
}

//------------------------------------------------------------------------------

SMClient::SMClient(const std::string& host, unsigned short port, const Timer& timer, MessageStream* msgStr) : SM(timer, msgStr), timersOffset(0.0) {
	boost::asio::ip::tcp::resolver resolver(io_service);
	boost::asio::ip::tcp::resolver::query query(boost::asio::ip::tcp::v4(), host, boost::lexical_cast<std::string>(port));
	boost::asio::ip::tcp::resolver::iterator iterator = resolver.resolve(query);
	
	boost::system::error_code error;
	tcpsocket.reset(new boost::asio::ip::tcp::socket(io_service));
	tcpsocket->connect(*iterator, error);
	if (error) {
		tcpsocket.reset();
		throw boost::system::system_error(error);
	}

	MessageStream::info(msgStr, "SMClient::SMClient(): host: %s, port: %i\n", tcpsocket->remote_endpoint().address().to_string().c_str(), tcpsocket->remote_endpoint().port());

	id = 0;
	exit = false;
	thread.reset(new boost::thread(&SMClient::run, this));
}

SMClient::~SMClient() {
	try {
		sendCommand(Command::TYPE_EXIT);
	}
	catch (const std::exception&) {}

	try {
		if (thread) {
			exit = true;
			thread->join();
		}
	}
	catch (const std::exception&) {}
	
	try {
		if (tcpsocket) {
			tcpsocket->close();
		}
	}
	catch (const std::exception&) {}
}

void SMClient::start() {
	sendCommand(Command::TYPE_DATA_START);
}

void SMClient::stop() {
	sendCommand(Command::TYPE_DATA_STOP);
}

void SMClient::syncTimers(unsigned syncTries, double syncWait) {
	if (!tcpsocket)
		throw std::runtime_error("SMClient::syncTimers(): NULL tcp socket");

	boost::asio::ip::udp::socket udpsocket(io_service);
	udpsocket.open(boost::asio::ip::udp::v4());
	boost::asio::ip::udp::endpoint recvpoint(tcpsocket->remote_endpoint().address(), tcpsocket->remote_endpoint().port());

	typedef std::map<double, double> Rank;
	Rank rank;
	
	for (unsigned i = 0; i < syncTries; ++i) {
		Sync syncout(i, timer.elapsed()), syncin;
		
		if (udpsocket.send_to(boost::asio::buffer(&syncout, sizeof(Sync)), recvpoint) != sizeof(Sync))
			throw std::runtime_error("SMClient::syncTimers(): unable to send message");

		boost::asio::ip::udp::endpoint sendendpoint;
		if (udpsocket.receive_from(boost::asio::buffer(&syncin, sizeof(Sync)), sendendpoint) != sizeof(Sync))
			throw std::runtime_error("SMClient::syncTimers(): unable to receive message");
		if (recvpoint.address().to_string().compare(sendendpoint.address().to_string()) || recvpoint.port() != sendendpoint.port())
			throw std::runtime_error("SMClient::syncTimers(): invalid sender address");
		if (syncin.id != syncout.id)
			throw std::runtime_error("SMClient::syncTimers(): unknown packet id");

		const double latency = 0.5*(timer.elapsed() - syncout.time);
		rank[latency] =  syncin.time - (syncout.time + latency);

		timer.sleep(syncWait);
	}

	if (rank.empty())
		throw std::runtime_error("SMClient::syncTimers(): empty latency map");

	// median with respect to latency
	size_t i = 0;
	Rank::const_iterator j = rank.begin();
	while (i < rank.size()/2) {++i;++j;}
	timersOffset = j->second;
	
	MessageStream::debug(msgStr, "SMClient::syncTimers(): latency=%f, timer offset=%f\n", j->first, j->second);
}

SMClient::Command SMClient::makeCommand(Command::Type type) const {
	if (exit)
		throw std::runtime_error("SMClient::makeCommand(): disconnected");
	if (!tcpsocket)
		throw std::runtime_error("SMClient::makeCommand(): NULL tcp socket");
	
	return Command(type);
}

void SMClient::sendCommand(Command::Type type) {
	if (type == Command::TYPE_DATA_ASYNC)
		throw std::runtime_error("SMClient::sendCommand(): async not supported");
	Command command = makeCommand(type);
	boost::asio::async_write(
		*tcpsocket,
		boost::asio::buffer(&command, sizeof(Command)),
		boost::asio::transfer_at_least(sizeof(Command)),
		boost::bind(
			&SMClient::commandHandler,
			this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred
		)
	);
}

void SMClient::commandHandler(const boost::system::error_code& error, size_t bytes_transferred) {
	if (error || bytes_transferred != sizeof(Command)) {
		MessageStream::error(msgStr, "SMClient::commandHandler(): error=%s, bytes=%u/%u\n", error.message().c_str(), (unsigned)bytes_transferred, (unsigned)sizeof(Command));
	}
}

void SMClient::write(unsigned size, const void* data) {
	Command command = makeCommand(Command::TYPE_DATA_ASYNC);
	
	Data buffer;
	buffer.resizeData(size);
	buffer.getHeader()->id = id++;
	buffer.getHeader()->time = toServerTime(timer.elapsed());
	std::copy((const char*)data, (const char*)data + buffer.getHeader()->size, (char*)buffer.getData());

	boost::array<boost::asio::mutable_buffer, 2> buffers = {
		boost::asio::buffer(&command, sizeof(Command)),
		boost::asio::buffer(buffer.get(), buffer.size())
	};
	boost::asio::async_write(
		*tcpsocket,
		buffers,
		boost::asio::transfer_at_least(sizeof(Command) + buffer.size()),
		boost::bind(
			&SMClient::dataHandler,
			this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred,
			sizeof(Command) + buffer.size()
		)
	);
}

void SMClient::dataHandler(const boost::system::error_code& error, size_t bytes_transferred, size_t bytes) {
	//MessageStream::debug(msgStr, "SMClient::dataHandler(): bytes=%u/%u\n", (unsigned)bytes_transferred, (unsigned)bytes);

	if (error || bytes_transferred != bytes) {
		MessageStream::error(msgStr, "SMClient::dataHandler(): error=%s, bytes=%u/%u\n", error.message().c_str(), (unsigned)bytes_transferred, (unsigned)bytes);
		return;
	}
}

void SMClient::recvData(Data& data) {
	if (!tcpsocket)
		throw std::runtime_error("SMClient::recvData(): NULL tcp socket");

	boost::system::error_code error;
	size_t size;

	size = boost::asio::read(
		*tcpsocket,
		boost::asio::buffer((void*)data.get(), sizeof(Header)),
#if (BOOST_VERSION >= 104700)
		boost::asio::transfer_exactly(sizeof(Header)),
#else
		boost::asio::transfer_at_least(sizeof(Header)),
#endif
		error
	);
	if (error || size < sizeof(Header))
		throw boost::system::system_error(error);

	data.resizeData(data.getHeader()->size);

	size = boost::asio::read(
		*tcpsocket,
		boost::asio::buffer(data.getData(), data.getHeader()->size),
		boost::asio::transfer_at_least(data.getHeader()->size),
		error
	);
	if (error || size < (size_t)data.getHeader()->size)
		throw boost::system::system_error(error);
}

void SMClient::run() {
	Data data;

	while (!exit) {
		try {
			recvData(data);
			data.getHeader()->time = toClientTime(data.getHeader()->time);
			SM::write(data);
		}
		catch (const std::exception& ex) {
			if (!exit) {
				// report error and stop
				MessageStream::error(msgStr, "SMClient::run(): %s\n", ex.what());
				exit = true;
				wakeup();
			}
		}
	}
}

//------------------------------------------------------------------------------
