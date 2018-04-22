#ifndef STRING_UTIL_H
#define STRING_UTIL_H

#include <map>
#include <list>
#include <vector>
#include <sstream>
#include <iomanip>
#include <stdlib.h>

#include <Golem/Ctrl/RobotJustin/my_exceptions.h>

#include <stdint.h>

using namespace std;

#define str_from_define(s) #s

typedef map<string, string> dict;

class py_value;

string repr(const char* input, int input_len);
string repr(string input);
string repr(py_value* input);
string repr(py_value& input);

template <class T> py_value* new_py_value(T v);
template <> py_value* new_py_value(string v);
template <> py_value* new_py_value(float v);
template <> py_value* new_py_value(double v);
template <> py_value* new_py_value(void* v);
template <> py_value* new_py_value(std::list<std::string> v);
template <> py_value* new_py_value(std::vector<double> v);

// template <class T> string get_repr(T v);
template <class T> string get_repr(T v) {
	py_value* pyv = new_py_value(v);
	string ret = repr(pyv);
	delete pyv;
	return ret;
}

class py_list;
class py_dict;

class py_value {
public:
	virtual ~py_value() { };

	virtual operator string() { return "<py_value>"; };
	virtual string repr() { return ::repr(string(*this)); };

	virtual operator bool() { return false; }
	virtual operator int() { throw str_exception_tb("can not cast py_value type %s to int! (%s)", type_name().c_str(), repr().c_str()); };
	virtual operator unsigned int() { return 0; }
	virtual operator float() { return 0.0; }
	virtual operator double() { return 0.0; }

	virtual string type_name() { return "py_value"; }
	virtual py_value* copy() { return new py_value(); }
	
	py_list* get_list();
	py_dict* get_dict();
};

class py_array : public py_value {
 public:
	int n_dims;
	int* shape;
	std::string dtype;
	int elem_size;
	void* data;

	py_array() {
		n_dims = 0;
		shape = NULL;
		data = NULL;
	};
	py_array(int n_dims, int* shape, std::string dtype, const void* data) 
		: n_dims(n_dims), dtype(dtype) {
		
		this->shape = (int*) malloc(n_dims * sizeof(int));
		unsigned int elems = 1;
		for(int i = 0; i < n_dims; i++) {
			elems *= shape[i];
			this->shape[i] = shape[i];
		}
		elem_size = 0;
		if(dtype == "float64")
			elem_size = 8;
		else if(dtype == "float32")
			elem_size = 4;
		this->data = malloc(elem_size * elems);
		memcpy(this->data, data, elem_size * elems);
	};
	virtual ~py_array() {
		if(shape)
			free(shape);
		if(data)
			free(data);
	};

	virtual operator string() {
		stringstream ss;
		ss << std::setprecision(15) << "array (dtype=" << dtype << ", shape=(";
		for(int i = 0; i < n_dims; i++) {
			ss << shape[i];
			if(i + 1 < n_dims)
				ss << ", ";
		}
		ss << "), data=0x" << data << ")";
		return ss.str();
	}
	virtual string repr() { return string(*this); };
	virtual string type_name() { return "py_array"; }
	virtual py_value* copy() { return new py_array(n_dims, shape, dtype, data); }
};



class py_string : public py_value {  
 public:
  string value;

  py_string() {};
  py_string(string v) : value(v) {};
  py_string(char* v, int len) { value = string(v, len); };
  py_string(const py_string& a) { value = a.value; }
  virtual ~py_string() {};

  virtual operator bool() { return value.size() > 0; }
  virtual operator string() { return value; }
  virtual operator int() { return atoi(value.c_str()); };
  virtual string type_name() { return "py_string"; }

  virtual py_value* copy() { return new py_string(value); }
};

class py_int : public py_value {
 public:
	int value;

  py_int() {};
  py_int(int v) : value(v) {};
  virtual ~py_int() {};

  virtual operator string() {
    stringstream ss;
    ss << value;
    return ss.str();
  }
  virtual string repr() { return string(*this); };
  virtual operator bool() { return value != 0; }
  virtual operator int() { return value; };
  virtual operator float() { return float(value); }
  virtual operator double() { return double(value); }
  virtual operator unsigned int() { return value; }
  virtual string type_name() { return "py_int"; }

  virtual py_value* copy() { return new py_int(value); }
};

typedef int64_t py_long_value_t;
class py_long : public py_value {
public:
	py_long_value_t value;

  py_long() {};
  py_long(py_long_value_t v) : value(v) {};
  virtual ~py_long() {};

  virtual operator string() {
    stringstream ss;
    ss << value << "L";
    return ss.str();
  }
  virtual string repr() { return string(*this); };
  virtual operator bool() { return value != 0; }
  virtual operator int() { return (int)value; };
  virtual operator py_long_value_t() { return value; };
  virtual operator float() { return float(value); }
  virtual operator double() { return double(value); }
  virtual operator unsigned int() { return (unsigned int)value; }
  virtual string type_name() { return "py_long"; }

  virtual py_value* copy() { return new py_long(value); }
};

class py_float : public py_value {
 public:
  double value;

  py_float() {};
  py_float(double v) : value(v) {};
  virtual ~py_float() {};

  virtual operator string() {
    stringstream ss;
    ss << std::setprecision(15) << value;
    return ss.str();
  }
  virtual string repr() { return string(*this); };
  virtual operator float() { return float(value); };
  virtual operator double() { return value; };
  virtual string type_name() { return "py_float"; }

  virtual py_value* copy() { return new py_float(value); }
};

class py_special : public py_value {
 public:
  string value;

  py_special() : value("None") {};
  py_special(string v) : value(v) {};
  virtual ~py_special() {};

  virtual operator bool() { return value == "True" || value == "true"; }
  virtual operator string() { return value; }
  virtual string repr() { return string(*this); };
  virtual string type_name() { return "py_special"; }

  virtual py_value* copy() { return new py_special(value); }
};

typedef vector<py_value*> py_list_value_t;
class py_list : public py_value {  
public:
	py_list_value_t value;

	virtual ~py_list() {
		for(py_list_value_t::iterator i = value.begin(); i != value.end(); i++)
			delete *i;
	}

	virtual operator string() {
		stringstream ss;
		ss << "[";
		for(py_list_value_t::iterator i = value.begin(); i != value.end(); i++) {
			if(i != value.begin())
				ss << ", ";
			ss << ::repr(*i);
		}
		ss << "]";
		return ss.str();
	}
	virtual string repr() { return string(*this); };
	virtual string type_name() { return "py_list"; }

	virtual py_value* copy() { 
		py_list* pyl = new py_list(); 
		for(py_list_value_t::iterator i = value.begin(); i != value.end(); i++)
			pyl->value.push_back((*i)->copy());
		return pyl;
	}

	py_value* operator[](int index) const {
		return value[index];
	}
	py_value* get(int index) const {
		return operator[](index);
	}
	unsigned int size() {
		return (unsigned int)value.size();
	}
	void append(py_value* l) {
		value.push_back(l);
	}
};


class py_tuple : public py_list {  
 public:

  virtual operator string() {
    stringstream ss;
    ss << "(";
    for(py_list_value_t::iterator i = value.begin(); i != value.end(); i++) {
      if(i != value.begin())
        ss << ", ";
      ss << ::repr(*i);
    }
    ss << ")";
    return ss.str();
  }
  virtual string repr() { return string(*this); };
  virtual string type_name() { return "py_tuple"; }
};


typedef map<string, py_value*> py_dict_value_t;
class py_dict : public py_value {  
 public:
  py_dict_value_t value;

  py_dict() {};
  py_dict(const py_dict& v) : value(v.value) {};
  virtual ~py_dict() {
    for(py_dict_value_t::iterator i = value.begin(); i != value.end(); i++)
      delete i->second;
  }

  py_value*& operator [](const string& v) {
    return value[v];
  }
  virtual operator string() {
    stringstream ss;
    ss << "{";
    for(py_dict_value_t::iterator i = value.begin(); i != value.end(); i++) {
      if(i != value.begin())
        ss << ", ";
      ss << ::repr(i->first) << ":" << ::repr(i->second);
    }
    ss << "}";
    return ss.str();
  }
  virtual string repr() { return string(*this); };
  virtual string type_name() { return "py_dict"; }

  virtual py_value* copy() { 
	  py_dict* pyd = new py_dict(); 
	  for(py_dict_value_t::iterator i = value.begin(); i != value.end(); i++)
		  pyd->value[i->first] = i->second->copy();
	  return pyd;
  }
};

string eval(string value);
string::size_type eval_string_until(string value, string::size_type start, string& output);
dict eval_dict(string value);
list<string> eval_list(string value);
py_value* eval_full(string value, string::size_type start=0, string::size_type* np_out=NULL);

unsigned int hex2dec(string hex);

string string_replace(string data, string search, string replace);
string strip(string input, string white="\r\n\t ");
string format_string(const char* format, ...);

list<string> split_command_line(string cmdline);
string join_command_line(list<string>& args);
vector<string> split_string(string input, string by, unsigned int max_split=0);
list<string> split(string what, string with, int n=0);

string join_path(string a, string b);
string get_dirname(string filename);
string get_basename(string filename);

#endif // STRING_UTIL_H
