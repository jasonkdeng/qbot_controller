#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
struct TestSerial {
  std::string output;
  std::string input;
  void begin(int) {}
  int available() const { return static_cast<int>(input.size()); }
  int read() { const char c = input.front(); input.erase(0, 1); return c; }
  void println(const char *text) { output += text; output += '\n'; }
  template<class... Args> void printf(const char *format, Args... args) {
    char buffer[1024];
    std::snprintf(buffer, sizeof(buffer), format, args...);
    output += buffer;
  }
};
inline TestSerial Serial;
