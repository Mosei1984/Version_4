#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

// Define the LogLevel enum
enum LogLevel {
    LOG_NONE,
    LOG_ERROR,
    LOG_WARNING,
    LOG_INFO,
    LOG_DEBUG,
    LOG_VERBOSE
};

// Enable or disable debug output globally
#define DEBUG_ENABLED true

// Debug macros
#if DEBUG_ENABLED
  #define DEBUG_PRINT(x)    Debug::print(x)
  #define DEBUG_PRINTLN(x)  Debug::println(x)
  #define DEBUG_PRINTF(...) Debug::printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

namespace Debug {
    extern bool enabled;
    extern LogLevel currentLogLevel;
    
    void init(unsigned long baudRate = 115200);
    void update();
    
    // Basic print functions
    void print(const char* msg);
    void print(const String& msg);
    void print(int value);
    void print(float value, int decimals = 2);
    
    // Basic println functions
    void println(const char* msg);
    void println(const String& msg);
    void println(int value);
    void println(float value, int decimals = 2);
    
    // Printf-style formatting
    void printf(const char* format, ...);
    
    // Log functions
    void log(LogLevel level, const char* message);
    void error(const char* message);
    void warning(const char* message);
    void info(const char* message);
    void debug(const char* message);
    
    // Set the current log level
    void setLogLevel(LogLevel level);
}

#endif // DEBUG_H
