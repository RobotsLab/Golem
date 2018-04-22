#ifndef LINE_ASSEMBLER_H
#define LINE_ASSEMBLER_H

#include <string>
#include <string.h>

using namespace std;

class line_assembler {
  string incomplete_line;
  typedef void (*callback_t)(void*, string);
  void* instance;
  callback_t callback;
  bool return_newline;

 public:
  line_assembler(void* instance, callback_t callback) {
    this->instance = instance;
    this->callback = callback;
    return_newline = false;
    reset();
  }

  void set_return_newline(bool return_newline) {
    this->return_newline = return_newline;
  }
  
  void reset() {
    incomplete_line = "";
  }
  
  void write(char* data, int len) {
    char* p = data;
    // printf("la write: %s len %d\n", repr(data, len).c_str(), len);
    while (p < data + len) {
      // printf("remaining: %s\n", repr(p, (data + len) - p).c_str());
      char* np = (char*)memchr(p, '\n', (data + len) - p);
      if (!np) {
        // this line is not completed                                                                             
        incomplete_line.append(p, len - (p - data));
        return; // wait for more data                                                                              
      }
      // got complete line from p to np
      string line = incomplete_line;
      if(return_newline)
        line.append(p, np - p + 1);
      else
        line.append(p, np - p);
      callback(instance, line);
      incomplete_line = "";
      p = np + 1;
    }
  }
};

#endif // LINE_ASSEMBLER_H
