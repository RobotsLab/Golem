#include <stdio.h>
#include <sstream>
#include <stdarg.h>

#include <Golem/Ctrl/RobotJustin/my_exceptions.h>
#include <Golem/Ctrl/RobotJustin/string_util.h>

py_list* py_value::get_list() {
	py_list* l = dynamic_cast<py_list*>(this);
	if(!l)
		throw str_exception("this py_value %s is not a list!\n", this->repr().c_str());
	return l;
}
py_dict* py_value::get_dict() {
	py_dict* l = dynamic_cast<py_dict*>(this);
	if(!l)
		throw str_exception("this py_value %s is not a dict!\n", this->repr().c_str());
	return l;
}


string repr(const char* input, size_t input_len) {
	stringstream ss;
	ss << "'";
	for(const char* cp = input; cp != input + input_len; cp++) {
		if((strchr(" ,.:;/_-+=[](){}^!?$", *cp) || isdigit(*cp) || isalpha(*cp)) && (unsigned char)*cp <= 127 && *cp) {
			//log("printable char: %d %c", *cp, *cp);
			ss << *cp;
			continue;
		}
		if(*cp == '\r') ss << "\\r";
		else if(*cp == '\n') ss << "\\n";
		else if(*cp == '<') ss << "<";
		else if(*cp == '>') ss << ">";
		else if(*cp == '\t') ss << "\\t";
		else if(*cp == '\\') ss << "\\\\";
		else if(*cp == '\'') ss << "\\'";
		else if(*cp == '"') ss << "\"";
		else if(*cp == '|') ss << "|";
		else if(*cp == '%') ss << "%";
		else {
			char format[16];
			_snprintf(format, 16, "\\x%02x", (unsigned char)*cp);
			ss << format;
		}
	}	
	ss << "'";
	return ss.str();
}

string repr(string input) {
	return repr(input.c_str(), input.size());
}

string repr(py_value* input) {
	if(!input)
		return "NULL";
	return input->repr();
	
	py_dict* d = dynamic_cast<py_dict*>(input);
	if(d)
		return /*string("dict:") + */string(*d);
	py_list* l = dynamic_cast<py_list*>(input);
	if(l)
		return /*string("list:") + */string(*l);
	py_int* i = dynamic_cast<py_int*>(input);
	if(i)
		return /*string("int:") + */string(*i);
	py_long* li = dynamic_cast<py_long*>(input);
	if(li)
		return /*string("long:") + */string(*li);
	py_float* f = dynamic_cast<py_float*>(input);
	if(f)
		return /*string("int:") + */string(*f);
	py_special* s = dynamic_cast<py_special*>(input);
	if(s)
		return /*string("special:") + */string(*s);
	return /*string("other:") + */repr(string(*input));

}

string repr(py_value& input) {
	return input.repr();
}

unsigned int hex2dec(string hex) {
	char* hex_order = "0123456789abcdef"; // can not be const because strchr on qnx awaits non-const!
	unsigned int value = 0;
	unsigned int base = 1;
	for(string::size_type p = hex.size() - 1; ; p--) {
		char c = hex[p];
		char* cp = strchr(hex_order, c);
		if(!cp)
			throw str_exception_tb("invalid hex char '%c' in %s at pos %d", c, repr(hex).c_str(), p);
		int index = (int)(cp - hex_order);
		value += index * base;
		base *= 16;
		if(!p)
			return value;
	}
}

string eval(string value) {
	string output;
	eval_string_until(value, 0, output);
	return output;
}

string::size_type eval_string_until(string value, string::size_type start, string& output) {
	stringstream ss;
	string::size_type p = start;
	string::size_type tp;

	p = value.find_first_not_of(" \r\n\t", p);
	if(p == string::npos)
		throw str_exception_tb("only whitespaces found!");

	// printf("try to read string from %s - start: %d\n", repr(value.substr(p)).c_str(), start);
	char delim = value[p];
	if(delim != '\'' && delim != '"') {
		// no delim!
		// printf("no delim!\n");
		// search next non identifier char
		tp = value.find_first_of(" \r\n\t,:'\"}{[]()", p);
		output = value.substr(p, tp - p);
		// printf("got non delim string %s\n", repr(output).c_str());
		return tp - 1;
		// throw str_exception_tb("can not eval non-string value %s - not supported start delim %c!", 
		//repr(value).c_str(), delim);
	}
	p++;
	while(p < value.size() - 1) {
		// find next backslash
		// printf("find backslash starting from %s\n", value.substr(p).c_str());
		string::size_type np = value.find('\\', p);
		if(np == string::npos) {
			// no further backslashes!
			// printf("no further backslashed!\n");
			// see where this string terminates!
			tp = value.find(delim, p);
			if(tp == string::npos)
				throw str_exception_tb("error: terminating ' not found in %s!", repr(value).c_str());			
			ss << value.substr(p, tp - p);
			break;
		}
		// found a backslash. 
		// printf("found a backslash\n");

		// check if there is a terminating '
		tp = value.find(delim, p);
		if(tp < np && tp != string::npos) {
			// found terminating tick before backslash! - finish
			// printf("found terminating tick before before next backslash. adding this string: %s\n", value.substr(p, tp - p - 1).c_str());
			ss << value.substr(p, tp - p);
			break;
		}
		// printf("handle backslash!\n");
			
		ss << value.substr(p, np - p);
		// look at next char
		if(value[np + 1] == 'r') ss << "\r";
		else if(value[np + 1] == 'n') ss << "\n";
		else if(value[np + 1] == '0') ss << "\0";
		else if(value[np + 1] == 't') ss << "\t";
		else if(value[np + 1] == '\\') ss << "\\";
		else if(value[np + 1] == '\'') ss << "'";
		else if(value[np + 1] == '"') ss << "\"";
		else if(value[np + 1] == 'x') {
			// 2 digit hex number
			ss << (char)hex2dec(value.substr(np + 2, 2));
			np += 2; // skip these two chars
		} else
			throw str_exception_tb("unknown excape sequence starting from %s", repr(value.substr(p)).c_str());
		p = np + 2;
	}
	output = ss.str();
	return tp;
}

dict eval_dict(string value) {
	//printf("reading dict: %s\n", value.c_str());
	dict o;
	string::size_type p = 1;

	if(value[0] != '{' || value[value.size() - 1] != '}')
		throw str_exception_tb("can not eval non-dict value %s!",
							repr(value).c_str());
	
	while(p < value.size() - 1) {
		string v_key;
		string v_value;
		// read key
		//printf("try to read key from %s\n", repr(value.substr(p)).c_str());
		string::size_type np = eval_string_until(value, p, v_key);
		// printf("got key %s and pos %d\n", repr(v_key).c_str(), np);
		if(np == string::npos) // no further key-value pairs!
			break;
		p = np + 2;
		// read value
		//printf("try to read value from %s\n", repr(value.substr(p)).c_str());
		np = eval_string_until(value, p, v_value);
		if(np == string::npos) // error reading value!
			break;
		o[v_key] = v_value;
		p = np + 2;
	}
	/*
	printf("got this dict:\n");
	for(dict::iterator i = o.begin(); i != o.end(); i++) {
		printf(" %s: %s\n", repr(i->first).c_str(), repr(i->second).c_str());
	}
	*/
	return o;
}
list<string> eval_list(string value) {
	// printf("reading list: %s\n", value.c_str());
	list<string> o;
	string::size_type p = 1;

	if(!strip(value).size())
		return o;
	if(value[0] != '[' || value[value.size() - 1] != ']')
		throw str_exception_tb("can not eval non-list value %s!",
							repr(value).c_str());
	
	while(p < value.size() - 1) {
		string v_key;
		//printf("try to read key from %s\n", repr(value.substr(p)).c_str());
		string::size_type np = eval_string_until(value, p, v_key);
		// printf("got key %s and pos %d\n", repr(v_key).c_str(), np);
		if(np == string::npos) // no further key-value pairs!
			break;
		o.push_back(v_key);
		p = np + 2;
	}
	/*
	printf("got this list:\n");
	for(list<string>::iterator i = o.begin(); i != o.end(); i++) {
		printf(" %s\n", repr(*i).c_str());
	}
	*/
	return o;
}

py_value* eval_full(string value, string::size_type start, string::size_type* np_out) {
	py_value* output = NULL;
	string::size_type p = start;
	string::size_type np;

	p = value.find_first_not_of(" \r\n\t", p);
	if(p == string::npos)
		throw str_exception_tb("only whitespaces found!");

	// printf("\neval_full: %s\n", value.substr(p).c_str());
	if(value[p] == '{') {
		py_dict* d = new py_dict();
		output = d;
		p++;
		while(p < value.size()) { // todo: stop erlier!
			// read key
			//printf("try to read key from %s\n", repr(value.substr(p)).c_str());
			np = value.find_first_not_of(" \r\n\t", p);
			if(np == string::npos)
				throw str_exception_tb("unexpected eos - wanted } in %s", repr(value).c_str());
			if(value[np] == '}')
				break;

			py_value* name = eval_full(value, p, &np);
			//printf("got key %s and pos %d\n", repr(*name).c_str(), np);
			if(np == string::npos)
				break;
			np = value.find_first_not_of(" \r\n\t", np + 1);
			if(np == string::npos)
				throw str_exception_tb("unexpected eos - wanted : in %s", repr(value).c_str());
			if(value[np] != ':')
				throw str_exception_tb("expected : - got %c", value[np]);
			p = np + 1;
			// read value
			//printf("try to read value from %s\n", repr(value.substr(p)).c_str());
			py_value* v_value = eval_full(value, p, &np);
			if(np == string::npos) // error reading value!
				break;
			d->value[*name] = v_value;
			delete name;
			//printf("np is at %s\n", repr(value.substr(np)).c_str());			
			np = value.find_first_not_of(" \r\n\t", np + 1);
			if(np == string::npos)
				throw str_exception_tb("unexpected eos - wanted , or } in %s", repr(value).c_str());
			if(value[np] == '}')
				break;
			if(value[np] != ',')
				throw str_exception_tb("wanted } or ' - got %c in %s", value[np], repr(value).c_str());
			p = np + 1;
			//printf("np now is at %s\n", repr(value.substr(p)).c_str());
		}
	} else if(value.substr(p, 6) == "array(") {
		// parse as list
		output = eval_full(value, p + 6, &np);
		// todo search closing )!
		if(value[np] == ')')
			np += 1; // closing ) from array!
		else {
			// skip data_type and so on
			np = value.find(")", p);
			// np += 1;
		}
			
	} else if(value[p] == '[') {
		py_list* l = new py_list();
		output = l;
		p++;
		while(p < value.size()) { // todo: stop erlier!
			// read value
			// printf("try to read value from %s\n", repr(value.substr(p)).c_str());

			np = value.find_first_not_of(" \r\n\t", p);
			if(np == string::npos)
				throw str_exception_tb("unexpected eos - wanted ] in %s", repr(value).c_str());
			if(value[np] == ']')
				break;

			py_value* v_value = eval_full(value, np, &np);
			// printf("got value %s and pos %d\n", repr(*v_value).c_str(), np);
			l->value.push_back(v_value);
 			// printf("np is at %s\n", repr(value.substr(np)).c_str());
			np = value.find_first_not_of(" \r\n\t", np + 1);
			if(np == string::npos)
				throw str_exception_tb("unexpected eos - wanted , in %s", repr(value).c_str());
			if(value[np] == ']')
				break;
			if(value[np] != ',') {
				printf("eval result up-to-now: %s\n", repr(output).c_str());
				throw str_exception_tb("expected , - got %c. in string %s", value[np], repr(value.substr(np - 3, 15)).c_str());
			}
			p = np + 1;
			// printf("np now is at %s\n", repr(value.substr(np)).c_str());
		}
	} else if(value[p] == '(') {
		py_list* l = new py_tuple();
		output = l;
		p++;
		while(p < value.size()) { // todo: stop erlier!
			// read value
			//printf("try to read value from %s\n", repr(value.substr(p)).c_str());

			np = value.find_first_not_of(" \r\n\t", p);
			if(np == string::npos)
				throw str_exception_tb("unexpected eos - wanted ) in %s", repr(value).c_str());
			if(value[np] == ')')
				break;

			py_value* v_value = eval_full(value, p, &np);
			// printf("got value %s and pos %d\n", repr(*v_value).c_str(), np);
			l->value.push_back(v_value);
 			//printf("np is at %s\n", repr(value.substr(np)).c_str());
			np = value.find_first_not_of(" \r\n\t", np + 1);
			if(np == string::npos)
				throw str_exception_tb("unexpected eos - wanted , in %s", repr(value).c_str());
			if(value[np] == ')')
				break;
			if(value[np] != ',')
				throw str_exception_tb("expected , - got %c. in string %s", value[np], repr(value.substr(np - 3, 15)).c_str());
			p = np + 1;
			//printf("np now is at %s\n", repr(value.substr(np)).c_str());
		}
	} else if(value[p] == '"' || value[p] == '\'') {
		py_string* s = new py_string();
		output = s;
		// read value
		//printf("try to read string from %s\n", repr(value.substr(p)).c_str());
		np = eval_string_until(value, p, s->value);
	} else if(isdigit(value[p]) || strchr("+-.", value[p])) {
		string v;
		np = eval_string_until(value, p, v);
		if(v.find(".") != string::npos) {
			py_float* s = new py_float();
			output = s;
			// printf("float from '%s'\n", v.c_str());
			s->value = atof(v.c_str());
		} else if(v.substr(v.size() - 1, 1) == "L") {
			py_long* s = new py_long();
			output = s;
			s->value = atol(v.c_str());
		} else {
			py_int* s = new py_int();
			output = s;
			s->value = atoi(v.c_str());
		}
	} else if(value.substr(p, 4) == "True" || value.substr(p, 5) == "False"|| value.substr(p, 4) == "None") {
		py_special* s = new py_special();
		output = s;
		np = eval_string_until(value, p, s->value);
	} else {
		//throw str_exception_tb("unknown python value %s\n", repr(value.substr(p)).c_str());
		py_string* s = new py_string();
		output = s;
		eval_string_until(value, p, s->value);
	}
	if(np_out)
		*np_out = np;
	//printf("terminating with np at %s\n",
	//repr(value.substr(np)).c_str());
	/*
	printf("recognized token %s in ->%s<-\n", repr(output).c_str(),
			value.substr(start, np - start).c_str());
	*/
	return output;
}


string string_replace(string data, string search, string replace) {
	string::size_type p = 0;
	while(p < data.size()) {
		string::size_type np = data.find(search, p);
		if(np == string::npos)
			break;
		data.replace(np, search.size(), replace);
		p = np + replace.size();
	}
	return data;
}

string strip(string input, string white) {
	string::size_type start = input.find_first_not_of(white);
	if(start == string::npos)
		return "";
	string::size_type end = input.find_last_not_of(white);
	return input.substr(start, end - start + 1);
}

static char* format_string_buffer = NULL;
void remove_format_string_buffer(void) {
	free(format_string_buffer);
}
string format_string(const char* format, ...) {
	/* Guess we need no more than 100 bytes. */
	int n;
	static int size = 0;
	static char*& p = format_string_buffer;
	char* np;

	if(!p) {
		size = 255;
		if ((p = (char*)malloc(size)) == NULL)
			throw std::bad_alloc();
		atexit(remove_format_string_buffer);
	}
	va_list ap;
	va_start(ap, format);
	
	while(true) {
		/* Try to print in the allocated space. */
		// va_start(ap, format);
		n = vsnprintf(p, size, format, ap);
		// va_end(ap);
		/* If that worked, return the string. */
		if (n > -1 && n < size)
			break;
		/* Else try again with more space. */
		if (n > -1)    /* glibc 2.1 */
			size = n+1; /* precisely what is needed */
		else           /* glibc 2.0 */
			size *= 2;  /* twice the old size */
		if ((np = (char*)realloc(p, size)) == NULL)
			throw std::bad_alloc();
		p = np;
	}

	va_end(ap);
	return string(p);
}

list<string> split_command_line(string cmd) {
	list<string> l;
	string::size_type p = 0;
	while(p != string::npos && p < cmd.size()) {
		string::size_type np;
		// skip eading spaces
		p = cmd.find_first_not_of(" ", p);
		if(p == string::npos)
			break;
		// test first char of this argument!
		if(cmd[p] != '"' && cmd[p] != '\'') {
			np = cmd.find(' ', p);
			l.push_back(cmd.substr(p, np - p));
			p = np;
			continue;
		}
		char delim = cmd[p];
		string::size_type end_delim_start = p + 1;

		// search end of string!
		while(true) {
			np = cmd.find(delim, end_delim_start);
			if(np == string::npos)
				throw str_exception_tb("invalid command line! quoted argument %d is not terminated by a quote!", l.size() + 1);
			string::size_type termpos = np - 1;
			unsigned int backslash_count = 0;
			// TODO: backslash handling not finished! need to unescape!!
			while(cmd[termpos] == '\\') {
				backslash_count ++;
				termpos --;
			}
			if((backslash_count % 2) == 0)
				break;
			end_delim_start = np + 1;
			if(end_delim_start >= cmd.size())
				throw str_exception_tb("invalid command line! quoted argument %d is not terminated by a quote!", l.size() + 1);
		}
		l.push_back(cmd.substr(p + 1, np - p - 1));
		p = np + 1;
	}
	return l;
}

string join_command_line(list<string>& args) {
	stringstream ss;
	for(list<string>::iterator i = args.begin(); i != args.end(); i++) {
		if(i != args.begin())
			ss << " ";
		ss << "\"" << *i << "\"";
	}
	return ss.str();
}

vector<string> split_string(string input, string by, unsigned int max_split) {
	vector<string> out;

	string::size_type s = 0;
	while(s < input.size()) {
		string::size_type p = input.find(by, s);
		if(p == string::npos) {
			out.push_back(input.substr(s));
			break;
		}
		out.push_back(input.substr(s, p - s));
		s = p + by.size();

		if(max_split > 0 && out.size() == max_split) {
			out.push_back(input.substr(s));
			break;
		}
	}
	return out;
}

list<string> split(string what, string with, int n) {
	list<string> l;
	string::size_type p = 0;
	while(p != string::npos && p < what.size()) {
		string::size_type np;
		np = what.find(with, p);
		if(np == string::npos)
			break;
		l.push_back(what.substr(p, np - p));
		p = np + 1;
		if((signed)l.size() == n)
			break;
	}
	l.push_back(what.substr(p));
	return l;
}

string get_dirname(string filename) {
	string::size_type p = filename.rfind("/");
	if(p == string::npos)
		return ".";
	return filename.substr(0, p);
}

string get_basename(string filename) {
	string::size_type p = filename.rfind("/");
	if(p == string::npos)
		return filename;
	return filename.substr(p + 1);
}

string join_path(string a, string b) {
	if(b.substr(0, 1) == "/")
		return b;

	if(b == "." || b == "./")
		return a;

	if(a == "")
		a = ".";
	if(a.substr(a.size() - 1, 1) != "/")
		a += "/";

	return a + b;
}

template <class T> py_value* new_py_value(T v) {
	return new py_string("<unknown c++ type>");
}

template <> py_value* new_py_value(string v) {
	return new py_string(v);
}
template <> py_value* new_py_value(float v) {
	return new py_float(v);
}
template <> py_value* new_py_value(double v) {
	return new py_float(v);
}
template <> py_value* new_py_value(void* v) {
	return new py_long((unsigned long)v);
}
template <> py_value* new_py_value(std::list<std::string> v) {
	py_list* l = new py_list();
	for(std::list<std::string>::iterator i = v.begin(); i != v.end(); i++) {
		l->value.push_back(new_py_value(*i));
	}
	return l;
}

template <> py_value* new_py_value(std::vector<double> v) {
	py_list* l = new py_list();
	for(std::vector<double>::iterator i = v.begin(); i != v.end(); i++) {
		l->value.push_back(new_py_value(*i));
	}
	return l;
	
}

