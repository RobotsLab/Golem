#ifndef BHAM_PLANNER_INTERFACE
#define BHAM_PLANNER_INTERFACE

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include <string>
#include <vector>
#include <list>
#include <map>

#include <Golem/Ctrl/RobotJustin/util.h>
#include <Golem/Ctrl/RobotJustin/bham_telemetry_packet.h>

namespace bham_planner_interface {
#ifdef NONSINDENT
}
#endif

class message_stream {
public:
	virtual ~message_stream() {}
	
	#define BHAM_PLANNER_INTERFACE_MESSAGE_STREAM(NAME, FILE)\
		virtual void NAME(const char* format, va_list argptr) const {\
			vfprintf(FILE, format, argptr);\
		}\
		static void NAME(message_stream* msg_str, const char* format, ...) {\
			if (msg_str) {\
				va_list argptr;\
				va_start(argptr, format);\
				msg_str->NAME(format, argptr);\
				va_end(argptr);\
			}\
		}

	BHAM_PLANNER_INTERFACE_MESSAGE_STREAM(debug, stdout)
	BHAM_PLANNER_INTERFACE_MESSAGE_STREAM(info, stdout)
	BHAM_PLANNER_INTERFACE_MESSAGE_STREAM(error, stderr)

	#undef BHAM_PLANNER_INTERFACE_MESSAGE_STREAM
};

typedef std::vector<float> config_t;
typedef std::map<std::string, config_t> config_dict_t;
typedef std::list<config_t> path_t;
class connection;

class frame_t {
public:
	double mat[12]; // row major 3x4

	frame_t() {
		// set identity
		memset(mat, 0, 12 * sizeof(double));
		for(unsigned int i = 0; i < 3; i++)
			mat[i * 4 + i] = 1;
	}

	double& operator[](size_t i) {
		return mat[i];
	}

	void print(message_stream* msg_str) {
		for(unsigned int r = 0; r < 3; r++) {
			for(unsigned int c = 0; c < 4; c++) {
				message_stream::info(msg_str, "%+6.2f ", mat[r * 4 + c]);
			}
			message_stream::info(msg_str, "\n");
		}
	}
};

class scene_object_t {
public:
	std::string class_name;
	std::string geometry_file;
	frame_t frame;
	std::string object_id;

	scene_object_t() {};
	scene_object_t(std::string class_name, std::string geometry_file, frame_t frame, std::string object_id)
		: class_name(class_name), geometry_file(geometry_file), frame(frame), object_id(object_id) {};

	void print(message_stream* msg_str) {
		message_stream::info(msg_str, "object_class: %s\n", class_name.c_str());
		message_stream::info(msg_str, "geometry file: %s\n", geometry_file.c_str());
		message_stream::info(msg_str, "frame:\n");
		frame.print(msg_str);
		message_stream::info(msg_str, "object_id: %s\n", object_id.c_str());
	}
};

class hand_contact_t {
public:
	std::string link_name;
	// contact in origin and rotation of object:
	double origin[3]; 
	double normal[3];
};

class request_t {
public:
	request_t() {};
	virtual ~request_t() {};

	virtual void print(message_stream* msg_str) = 0;
	
	virtual void set(std::string name, float value) = 0;
	virtual void set(std::string name, std::string value) = 0;
	virtual void set(std::string name, std::vector<std::string> value) = 0;
	virtual void set(std::string name, std::vector<float> value) = 0;
	virtual void set(std::string name, std::vector<double> value) = 0;
	virtual void set(std::string name, std::list<std::string> value) = 0;
	virtual void set(std::string name, std::list<float> value) = 0;
	virtual void set(std::string name, std::list<double> value) = 0;
	virtual void set(std::string name, std::map<std::string, float> value) = 0;
	virtual void set(std::string name, std::map<std::string, std::string> value) = 0;
	virtual void set(std::string name, path_t value) = 0;

	virtual std::string get(std::string name, std::string default_value="") = 0;
	virtual int get_int(std::string name) = 0;
	virtual double get_double(std::string name) = 0;
	virtual config_t get_config(std::string name) = 0;
	virtual config_dict_t get_config_dict(std::string name) = 0;
	virtual std::vector<int> get_vector_of_ints(std::string name) = 0;
	virtual std::list<scene_object_t> get_list_of_objects(std::string name) = 0;
	virtual frame_t get_frame(std::string name) = 0;

	virtual void get_array(std::string name, void** data, std::string* dtype, int* elem_size, int* n_dims, int** shape) = 0;
	
	virtual std::list<hand_contact_t> get_contacts(std::string name) = 0;

	virtual bool has(std::string name) = 0;

	virtual void clear() = 0;

	virtual request_t* copy() = 0;

	static request_t* create();
};

class connection {
public:
	typedef request_t* (*request_handler_t)(connection*, request_t*);
	typedef void (*udp_receiver_t)(connection*, telemetry_packet*);

	virtual void connect() = 0;
	virtual bool is_connected() = 0;
	virtual void disconnect() = 0;
	
	virtual void register_handler(std::string request_name, request_handler_t) = 0;
	virtual void register_udp_receiver(udp_receiver_t receiver, int port=54454) = 0;
	
	virtual void run() = 0;
	virtual void quit() = 0;
	virtual bool iterate(unsigned timeout_msec = -1) = 0;

	virtual void send_request(request_t* r) = 0;
	virtual request_t* wait_for_request(const std::string& request, unsigned timeout_msec = -1) = 0;
	virtual request_t* send_and_wait_for_request(request_t* r) = 0;

	static connection* create(std::string hostname="localhost", int port=54363, message_stream* msg_str = NULL);
	virtual ~connection() {};
};

};

#endif // BHAM_PLANNER_INTERFACE
