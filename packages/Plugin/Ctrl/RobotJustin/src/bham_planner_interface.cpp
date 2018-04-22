#include <Golem/Ctrl/RobotJustin/bham_planner_interface.h>

#include <map>
#include <sstream>

#include <Golem/Ctrl/RobotJustin/string_util.h>
#include <Golem/Ctrl/RobotJustin/my_exceptions.h>
#include <Golem/Ctrl/RobotJustin/line_assembler.h>
#include <Golem/Ctrl/RobotJustin/util.h>

using namespace std;
using namespace bham_planner_interface;


#ifdef __LINUX__
void timespecnorm(timespec& ts) {
	if (ts.tv_nsec >= 1000000000L) {
		++ts.tv_sec;
		ts.tv_nsec -= 1000000000L;
	}
	if (ts.tv_nsec < 0) {
		--ts.tv_sec;
		ts.tv_nsec += 1000000000L;
	}
}
unsigned timespec_to_msec(const timespec& ts) {
	return unsigned(ts.tv_sec) > unsigned(-1)/1000 || unsigned(ts.tv_sec)*1000 > unsigned(-1) - ts.tv_nsec/1000000 ? unsigned(-1) : unsigned(ts.tv_sec)*1000 + ts.tv_nsec/1000000;
}
#endif	// __LINUX__

class msec_timer {
private:
#ifdef __WIN32__
	unsigned stamp;
#else // __LINUX__
	timespec stamp;
#endif

public:
	msec_timer() {
#ifdef __WIN32__
		stamp = ::timeGetTime();
#else // __LINUX__
		::clock_gettime(CLOCK_REALTIME, &stamp);
#endif
	}
	unsigned elapsed() const {
#ifdef __WIN32__
		return ::timeGetTime() - stamp;
#else // __LINUX__
		struct timespec ts;
		::clock_gettime(CLOCK_REALTIME, &ts);
		ts.tv_sec -= stamp.tv_sec;
		ts.tv_nsec -= stamp.tv_nsec;
		timespecnorm(ts);
		return timespec_to_msec(ts);
#endif
	}
};


class impl_connection;

frame_t get_frame(py_value* v) {
	py_list* rows = dynamic_cast<py_list*>(v);
	if(!rows)
		throw str_exception_tb("get_frame(): it's not a list!");
	if(rows->size() < 3)
		throw str_exception("frame needs to have atleast 3 rows!");
	frame_t frame;
	for(unsigned int r = 0; r < 3; r++) {
		py_list* row = rows->get(r)->get_list();
		frame[r * 4 + 3] = *row->get(3);
		for(unsigned int c = 0; c < row->size(); c++)
			frame[r * 4 + c] = *row->get(c);
	}
	return frame;
}

class impl_request_t : public request_t {
	typedef std::map<string, py_value*> values_t;
	values_t values;
	
public:
	impl_request_t() {
	}
	virtual ~impl_request_t() {
		clear();
	}
	virtual void print(message_stream* msg_str) {
		for(values_t::iterator i = values.begin(); i != values.end(); i++) {
			message_stream::info(msg_str, "  %s: %s\n", i->first.c_str(), repr(i->second).c_str());
		}
	}
	
	void set(string name, py_value* value) {
		values[name] = value;
	}
	virtual void set(string name, float value) {
		values[name] = new py_float(value);
	}
	virtual void set(string name, string value) {
		values[name] = new py_string(value);
	}
	virtual void set(string name, vector<float> value) {
		py_list* l = new py_list();
		for(vector<float>::iterator i = value.begin(); i != value.end(); i++)
			l->value.push_back(new_py_value(*i));
		values[name] = l;
	}
	virtual void set(string name, vector<double> value) {
		py_list* l = new py_list();
		for(vector<double>::iterator i = value.begin(); i != value.end(); i++)
			l->value.push_back(new_py_value(*i));
		values[name] = l;
	}
	virtual void set(string name, vector<string> value) {
		py_list* l = new py_list();
		for(vector<string>::iterator i = value.begin(); i != value.end(); i++)
			l->value.push_back(new_py_value(*i));
		values[name] = l;
	}
	virtual void set(string name, list<string> value) {
		py_list* l = new py_list();
		for(list<string>::iterator i = value.begin(); i != value.end(); i++)
			l->value.push_back(new_py_value(*i));
		values[name] = l;
	}
	virtual void set(string name, list<float> value) {
		py_list* l = new py_list();
		for(list<float>::iterator i = value.begin(); i != value.end(); i++)
			l->value.push_back(new_py_value(*i));
		values[name] = l;
	}
	virtual void set(string name, list<double> value) {
		py_list* l = new py_list();
		for(list<double>::iterator i = value.begin(); i != value.end(); i++)
			l->value.push_back(new_py_value(*i));
		values[name] = l;
	}
	virtual void set(string name, path_t value) {
		py_list* lp = new py_list();
		for(path_t::iterator i = value.begin(); i != value.end(); i++) {
			py_list* lc = new py_list();
			for(config_t::iterator k = i->begin(); k != i->end(); k++) {
				lc->value.push_back(new_py_value(*k));
			}
			lp->value.push_back(lc);
		}
		values[name] = lp;
	}
	virtual void set(string name, map<string, float> value) {
		py_dict* l = new py_dict();
		for(map<string, float>::iterator i = value.begin(); i != value.end(); i++)
			l->value[i->first] = new_py_value(i->second);
		values[name] = l;
	}

	virtual void set(string name, map<string, string> value) {
		py_dict* l = new py_dict();
		for(map<string, string>::iterator i = value.begin(); i != value.end(); i++)
			l->value[i->first] = new_py_value(i->second);
		values[name] = l;
	}

	virtual string get(string name, string default_value="") {
		values_t::iterator i = values.find(name);
		if(i == values.end())
			return default_value;
		return string(*i->second);
	}
	virtual int get_int(string name) {
		values_t::iterator i = values.find(name);
		if(i == values.end())
			throw str_exception_tb("there is no %s member in this request!", name.c_str());
		return int(*i->second);
	}
	virtual double get_double(string name) {
		values_t::iterator i = values.find(name);
		if(i == values.end())
			throw str_exception_tb("there is no %s member in this request!", name.c_str());
		return double(*i->second);
	}
	virtual config_t get_config(std::string name) {
		values_t::iterator i = values.find(name);
		if(i == values.end())
			throw str_exception_tb("there is no %s member in this request!", name.c_str());
		py_list* l = dynamic_cast<py_list*>(i->second);
		if(!l)
			throw str_exception_tb("member %s is not a list!", name.c_str());
		config_t conf;
		for(py_list_value_t::iterator i = l->value.begin(); i != l->value.end(); i++)
			conf.push_back((float)**i);
		return conf;
	}
	virtual config_dict_t get_config_dict(std::string name) {
		values_t::iterator i = values.find(name);
		if(i == values.end())
			throw str_exception_tb("there is no %s member in this request!", name.c_str());
		py_dict* d = dynamic_cast<py_dict*>(i->second);
		if(!d)
			throw str_exception_tb("member %s is not a dict!", name.c_str());
		config_dict_t dict;
		for(py_dict_value_t::iterator k = d->value.begin(); k != d->value.end(); k++) {
			py_list* l = dynamic_cast<py_list*>(k->second);
			if(!l)
				throw str_exception_tb("config-dict member %s is not a list!", k->first.c_str());
			config_t conf;
			for(py_list_value_t::iterator j = l->value.begin(); j != l->value.end(); j++)
				conf.push_back((float)**j);
			dict[k->first] = conf;
		}
		return dict;
	}

	virtual vector<int> get_vector_of_ints(string name) {
		values_t::iterator i = values.find(name);
		if(i == values.end())
			throw str_exception_tb("there is no %s member in this request!", name.c_str());
		py_list* l = dynamic_cast<py_list*>(i->second);
		if(!l)
			throw str_exception_tb("member %s is not a list!", name.c_str());
		vector<int> conf;
		for(py_list_value_t::iterator i = l->value.begin(); i != l->value.end(); i++)
			conf.push_back((int)**i);
		return conf;
	}

	virtual list<scene_object_t> get_list_of_objects(string name) {
		list<scene_object_t> ret;
		values_t::iterator i = values.find(name);
		if(i == values.end())
			throw str_exception_tb("there is no %s member in this request!", name.c_str());
		py_list* l = dynamic_cast<py_list*>(i->second);
		if(!l)
			throw str_exception_tb("member %s is not a list!", name.c_str());
		unsigned int n = 0;
		for(py_list_value_t::iterator i = l->value.begin(); i != l->value.end(); i++) {
			py_list* o = dynamic_cast<py_list*>(*i);
			if(!o)
				throw str_exception_tb("objects-list %s item %d is not a list!", name.c_str(), n);
			n++;

			ret.push_back(scene_object_t(
							  *o->get(0),
							  *o->get(1),
							  ::get_frame(o->get(2)),
							  *o->get(3)));
		}

		return ret;
	}

	virtual void get_array(string name, void** data, string* dtype, int* elem_size, int* n_dims, int** shape) {
		values_t::iterator i = values.find(name);
		if(i == values.end())
			throw str_exception_tb("there is no %s member in this request!", name.c_str());
		py_array* a = dynamic_cast<py_array*>(i->second);
		if(!a)
			throw str_exception_tb("member %s is not an array!", name.c_str());
		if(data)
			*data = a->data;
		if(dtype)
			*dtype = a->dtype;
		if(elem_size)
			*elem_size = a->elem_size;
		if(n_dims)
			*n_dims = a->n_dims;
		if(shape)
			*shape = a->shape;
	}

	virtual list<hand_contact_t> get_contacts(string name) {
		list<hand_contact_t> contacts;
		values_t::iterator i = values.find(name);
		if(i == values.end())
			throw str_exception_tb("there is no %s member in this request!", name.c_str());
		py_list* py_contacts = dynamic_cast<py_list*>(i->second);
		if(!py_contacts)
			throw str_exception_tb("member %s is not a list!", name.c_str());

		for(py_list_value_t::iterator i = py_contacts->value.begin(); i != py_contacts->value.end(); ++i) {
			py_list* py_contact = dynamic_cast<py_list*>(*i);
			if(!py_contact)
				throw str_exception_tb("item in contacts-list is not a list!");
			if(py_contact->value.size() != (1 + 3 + 3))
				throw str_exception_tb("item in contacts-list has %d items instead of 1+3+3=7", py_contact->value.size());
			py_list_value_t::iterator k = py_contact->value.begin();
			hand_contact_t c;
			py_string* py_link = dynamic_cast<py_string*>(*k++);
			if(!py_link)
				throw str_exception_tb("first item in contact-list is not a string!");
			c.link_name = py_link->value;
			for(unsigned int j = 0; j < 3; j++) {
				py_float* f = dynamic_cast<py_float*>(*k++);
				if(!f)
					throw str_exception_tb("item %d in origin part of contact-list is not a float!", j + 1);
				c.origin[j] = f->value;
			}
			for(unsigned int j = 0; j < 3; j++) {
				py_float* f = dynamic_cast<py_float*>(*k++);
				if(!f)
					throw str_exception_tb("item %d in normal part of contact-list is not a float!", j + 1 + 3);
				c.normal[j] = f->value;
			}
			contacts.push_back(c);
		}
		return contacts;
	}

	virtual frame_t get_frame(string name) {
		values_t::iterator i = values.find(name);
		if(i == values.end())
			throw str_exception_tb("there is no %s member in this request!", name.c_str());
		py_list* a = dynamic_cast<py_list*>(i->second);
		if(!a)
			throw str_exception_tb("member %s is not an list/frame!", name.c_str());
		return ::get_frame(a);
	}

	virtual bool has(string name) {
		return values.find(name) != values.end();
	}

	virtual void clear() {
		for(values_t::iterator i = values.begin(); i != values.end(); i++)
			delete i->second;
		values.clear();
	}
	virtual request_t* copy() {
		impl_request_t* c = new impl_request_t();
		for(values_t::iterator i = values.begin(); i != values.end(); i++)
			c->set(i->first, i->second->copy());
		return c;
	}

private:
	friend class impl_connection;
	string serialize() {
		stringstream ss;
		for(values_t::iterator i = values.begin(); i != values.end(); i++) {
			ss << i->first << ": " << (i->second)->repr() << "\n";
		}
		ss << "\n";
		return ss.str();
	}
};

request_t* request_t::create() {
	return new impl_request_t();
}

class impl_connection : public connection {
	message_stream* msg_str;

	string hostname;
	int port;
	bool connected;

	typedef std::map<std::string, request_handler_t> request_handlers_t;
	request_handlers_t request_handlers;

	int fd;
	line_assembler la;
	int data_len;
	string data_key;
	string data_value;
	int skip_empty_lines;
	impl_request_t received_request;

	typedef map<string, impl_request_t*> awaited_requests_t;
	awaited_requests_t awaited_requests;

	udp_receiver_t udp_receiver;
	int udp_receiver_port;
	int udp_receiver_socket;
	telemetry_packet last_telemetry_packet;
public:
	impl_connection(std::string hostname, int port, message_stream* msg_str) : hostname(hostname), port(port), msg_str(msg_str), la(this, &received_line) {
		connected = false;
		fd = -1;
		udp_receiver = NULL;		
		udp_receiver_port = -1;
		udp_receiver_socket = -1;
#ifdef __WIN32__
		WSADATA wsaData = {0};
		DWORD iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (iResult != 0)
			throw str_exception_tb("WSAStartup failed: %d", iResult);
#endif
		data_len = 0;
		skip_empty_lines = 0;
		this->connect();
	}

	~impl_connection() {
		disconnect();
		for(awaited_requests_t::iterator i = awaited_requests.begin(); i != awaited_requests.end(); i++)
			if(i->second)
				delete i->second;
	}

	virtual void connect() {
		connected = true;
		fd = (int)socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
		if(fd == -1)
			throw errno_exception("socket");
		
		struct sockaddr_in addr;
		addr.sin_family = AF_INET;
		addr.sin_port = htons(port);
		
		if(resolve_hostname(hostname.c_str(), &addr) == -1)
			throw errno_exception_tb("can not resolve hostname: '%s'", hostname.c_str());
		
		if(::connect(fd, (struct sockaddr*)&addr, sizeof(addr)))
			throw errno_exception_tb("connect");
		
		connected = true;
	}

	virtual bool is_connected() { 
		return connected;
	}

	virtual void disconnect() {
		if(connected) {
			message_stream::info(msg_str, "bham_planner_interface disconnecting!\n");
			connected = false;
			shutdown(fd, SHUT_RDWR);
			#ifndef __WIN32__
			close(fd);
			#endif
		}
		fd = -1;
	}
	
	virtual void register_handler(string request_name, request_handler_t handler) {
		request_handlers[request_name] = handler;
	}
	virtual void register_udp_receiver(udp_receiver_t receiver, int port) {
		udp_receiver = receiver;
		udp_receiver_port = port;
		if(udp_receiver_socket != -1) {
			shutdown(udp_receiver_socket, SHUT_RDWR);
			#ifndef __WIN32__
			close(udp_receiver_socket);
			#endif
		}
		int fd = (int)socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
		udp_receiver_socket = fd;
		if(fd == -1)
			throw errno_exception("udp socket");

		struct sockaddr_in addr;
		addr.sin_family = AF_INET;
		addr.sin_port = htons(port);
		addr.sin_addr.s_addr = INADDR_ANY;
		
		if(::bind(fd, (struct sockaddr*)&addr, sizeof(addr)))
			throw errno_exception_tb("udp bind");		
	}


	bool keep_running;
	virtual void run() {
		keep_running = true;
		while(connected && keep_running) {
			iterate();
		}
	}
	virtual void quit() {
		keep_running = false;
	}

	virtual bool iterate(unsigned timeout_msec = -1) {
		if(!connected)
			throw str_exception_tb("no bham_planner_interface-connection!");
		
		fd_set read_fds;
		FD_ZERO(&read_fds);
		FD_SET(fd, &read_fds);

		int max_fd = fd;
		if(udp_receiver_socket != -1) {
			FD_SET(udp_receiver_socket, &read_fds);
			if(udp_receiver_socket > max_fd)
				max_fd = udp_receiver_socket;
		}

		timeval tv = {timeout_msec/1000, (timeout_msec%1000)*1000}, *timeout = timeout_msec == -1 ? NULL : &tv;
		int n = select(max_fd + 1, &read_fds, NULL, NULL, timeout);
		if (n == 0) // timeout
			return false;
		if (n == -1) // error
			throw str_exception_tb("select");
		
		if (FD_ISSET(udp_receiver_socket, &read_fds)) {
			// there is a new telemetry packet
			int n = recv(udp_receiver_socket, (char*)&last_telemetry_packet, sizeof(last_telemetry_packet), 0);
			if(n != sizeof(last_telemetry_packet)) {
				message_stream::error(msg_str, "wanted to receive UDP packet of size %d but recv() returned %d\n", sizeof(last_telemetry_packet), n);
				if (n == -1) {
					message_stream::info(msg_str, "closing UDP telemetry socket!\n");
					shutdown(udp_receiver_socket, SHUT_RDWR);
#ifndef __WIN32__
					close(udp_receiver_socket);
#endif
					udp_receiver_socket = -1;
				}
			} else 
				udp_receiver(this, &last_telemetry_packet);
			
		}
		if (FD_ISSET(fd, &read_fds)) {
			// there is data to read from socket
			char buffer[1024];
			n = recv(fd, buffer, 1024, 0);
			if(n == 0) {
				message_stream::info(msg_str, "got EOF on planner_interface connection!\n");
				disconnect();
				return true;
			}
			la.write(buffer, n);
		}

		return true;
	}

	virtual void send_request(request_t* r_) {
		impl_request_t* r = dynamic_cast<impl_request_t*>(r_);
		if(!r) {
			message_stream::error(msg_str, "error, this is not a impl_request_t!\n");
			return;
		}
		string s = r->serialize();
		message_stream::debug(msg_str, "sending: %s\n", s.c_str());
		int n = ::send(fd, s.c_str(), (int)s.size(), 0);
		if(n != (signed)s.size())
			message_stream::error(msg_str, "warning: wanted to send %d bytes. but send() returned %d\n", s.size(), n);
	}

	virtual request_t* wait_for_request(const std::string& request, unsigned timeout_msec = -1) {
		awaited_requests[request] = NULL; // register as awaited request/response!
		// setup timer
		msec_timer timer;
		// set the request if UDP packet has been read
		for (;;) {
			// read socket data within the timeout_msec
			iterate(timeout_msec);
			if (awaited_requests[request] != NULL) {
				request_t* ret = awaited_requests[request];
				if(ret->has("success") && string(ret->get("success")).substr(0, 5) == "error")
					throw str_exception_tb("request %s failed:\n%s", request.c_str(), string(ret->get("success")).c_str());
				awaited_requests.erase(request);
				return ret;
			}
			const unsigned elapsed = timer.elapsed();
			if (elapsed > timeout_msec)
				break;
			else
				timeout_msec -= elapsed;
		}
		return NULL;
	}

	virtual request_t* send_and_wait_for_request(request_t* r) {
		const std::string request = r->get("request");
		send_request(r);
		return wait_for_request(request);
	}

private:

	static void received_line(void* instance, string line) {
		impl_connection* self = (impl_connection*) instance;
		self->feed_line(line);
	}
	void feed_line(std::string line) {
		// message_stream::debug(msg_str, "received line: '%s'\n", line.c_str());
		// printf("received line: %s\n", repr(line).c_str());
		if(data_len) {
			// read further data
			line.append("\n");
			if(data_len >= (signed)line.size()) {
				data_value.append(line);
				data_len -= (int)line.size();
				line = "";
			} else {
				data_value.append(line.substr(0, data_len));
				line = line.substr(data_len + 1); // skip appended \n
				data_len = 0;
			}
			if(data_len == 0) {
				if(data_value.substr(0, 6) == "array ") {
					string::size_type pos = data_value.find("\n");
					string array_spec = data_value.substr(0, pos);
					unsigned int offset = (unsigned int)pos + 1;
					// unsigned int n_bytes = data_value.size() - offset;
					vector<string> spec = split_string(array_spec, " ");
					string dtype = spec[1];
					unsigned int n_dims = atoi(spec[2].c_str());
					int* shape = new int[n_dims];
					for(unsigned int i = 0; i < n_dims; i++)
						shape[i] = atoi(spec[3 + i].c_str());
					py_value* v = new py_array(n_dims, shape, dtype, (data_value.c_str() + offset));
					received_request.set(data_key, v);
					delete[] shape;
				} else 
					received_request.set(data_key, eval_full(data_value));
				if(data_value[data_value.size() - 1] == '\n')
					skip_empty_lines = 1;
				data_value = "";
			}
			if(line.size())
				message_stream::debug(msg_str, "warning: got remaining line %s", repr(line).c_str());
			return;
		}
		if(line == "") {
			if(skip_empty_lines) {
				skip_empty_lines--;
				return;
			}
			// process and finish this request
			try {	
				process_request();
			}
			catch(str_exception& e) {
				// fprintf(stderr, "uncaught str_exception while processing request:\n'%s'\n", e.what());
				message_stream::debug(msg_str, "uncaught str_exception while processing request:\n'%s'\n", e.what());
			}
			catch(exception& e) {
				message_stream::debug(msg_str, "uncaught exception while processing request:\n%s\n", e.what());
			}
			received_request.clear();
			return;
		}
		string::size_type p = line.find(':');
		if(p == string::npos) {
			message_stream::debug(msg_str, "invalid request line received: %s", repr(line).c_str());
			return;
		}

		string key;
		string value;
		key.assign(line, 0, p);
		p ++;
		while(line[p] == ' ' && p < line.size()) p++;
		value.assign(line, p, line.size() - p);
		if(value.substr(0, 5) == "data ") {
			// data block
			data_key = key;
			data_len = atoi(value.substr(5).c_str());
			data_value = "";
			return;
		}
		// message_stream::debug(msg_str, "input value: %s\n", value.c_str());
		// message_stream::debug(msg_str, "evaluated value: %s\n", value.c_str());
		received_request.set(key, eval_full(value));

	}

	void process_request() {
		// handlers have priority
		const bool hasRequest = received_request.has("request");
		if (hasRequest) {
			request_handlers_t::iterator i = request_handlers.find(received_request.get("request"));
			if (i != request_handlers.end()) {
				request_handler_t& handler = i->second;
				request_t* request = received_request.copy();
				received_request.clear();
				request_t* response = handler(this, request);
				delete request;
				if (response) {
					send_request(response);
					delete response;
				}
				return;
			}
		}
		// otherwise check if there is no awaited response or request
		const bool hasResponse = received_request.has("response");
		if (hasResponse || hasRequest) {
			string response = received_request.get(hasResponse ? "response" : "request");
			awaited_requests_t::iterator k = awaited_requests.find(response);
			if (k != awaited_requests.end()) {
				k->second = (impl_request_t*)received_request.copy();
				return;
			}
		}
		// neither request nor reponse!
		message_stream::debug(msg_str, "ignoring invalid %s:\n", hasRequest ? "request" : hasResponse ? "response" : "request/response");
		received_request.print(msg_str);
	}
};


connection* connection::create(std::string hostname, int port, message_stream* msg_str) {
	return new impl_connection(hostname, port, msg_str);
}
