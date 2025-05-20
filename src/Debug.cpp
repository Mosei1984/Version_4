#include "Debug.h"
#include <stdarg.h>

namespace Debug {
  // Extern-Variable Definition
  bool enabled = false;
  LogLevel currentLogLevel = LOG_INFO;

  void init(unsigned long baud) {
    #if DEBUG_ENABLED
    Serial.begin(baud);
    while (!Serial && millis() < 3000);
    enabled = true;
    Serial.println(F("Debug system initialized"));
    #endif
  }

  void update() {
    #if DEBUG_ENABLED
    if (!enabled) return;
    if (Serial.available()) {
      char cmd = Serial.read();
      switch (cmd) {
        case 'd': currentLogLevel = LOG_DEBUG;   Serial.println(F("Log level: DEBUG"));   break;
        case 'i': currentLogLevel = LOG_INFO;    Serial.println(F("Log level: INFO"));    break;
        case 'w': currentLogLevel = LOG_WARNING; Serial.println(F("Log level: WARNING")); break;
        case 'e': currentLogLevel = LOG_ERROR;   Serial.println(F("Log level: ERROR"));   break;
        case 'h':
          Serial.println(F("Commands: d=DEBUG, i=INFO, w=WARNING, e=ERROR, h=HELP"));
          break;
      }
      while (Serial.available()) Serial.read();
    }
    #endif
  }

  void log(LogLevel level, const char* message) {
    #if DEBUG_ENABLED
    if (!enabled || level > currentLogLevel) return;
    switch (level) {
      case LOG_ERROR:   Serial.print(F("[ERROR] ")); break;
      case LOG_WARNING: Serial.print(F("[WARN] "));  break;
      case LOG_INFO:    Serial.print(F("[INFO] "));  break;
      case LOG_DEBUG:   Serial.print(F("[DEBUG] ")); break;
      case LOG_VERBOSE: Serial.print(F("[VERB] "));  break;
      default: break;
    }
    Serial.println(message);
    #endif
  }

  void error(const char* message)   { log(LOG_ERROR,   message); }
  void warning(const char* message) { log(LOG_WARNING, message); }
  void info(const char* message)    { log(LOG_INFO,    message); }
  void debug(const char* message)   { log(LOG_DEBUG,   message); }

  void setLogLevel(LogLevel level) {
    currentLogLevel = level;
  }

  void print(const char* text) {
    #if DEBUG_ENABLED
    if (!enabled) return;
    Serial.print(text);
    #endif
  }

  void print(const String& text) {
    #if DEBUG_ENABLED
    if (!enabled) return;
    Serial.print(text);
    #endif
  }

  void print(int value) {
    #if DEBUG_ENABLED
    if (!enabled) return;
    Serial.print(value);
    #endif
  }

  void print(float value, int decimals) {
    #if DEBUG_ENABLED
    if (!enabled) return;
    Serial.print(value, decimals);
    #endif
  }

  void println(const char* text) {
    #if DEBUG_ENABLED
    if (!enabled) return;
    Serial.println(text);
    #endif
  }

  void println(const String& text) {
    #if DEBUG_ENABLED
    if (!enabled) return;
    Serial.println(text);
    #endif
  }

  void println(int value) {
    #if DEBUG_ENABLED
    if (!enabled) return;
    Serial.println(value);
    #endif
  }

  void println(float value, int decimals) {
    #if DEBUG_ENABLED
    if (!enabled) return;
    Serial.println(value, decimals);
    #endif
  }

  void printf(const char* format, ...) {
    #if DEBUG_ENABLED
    if (!enabled) return;
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial.print(buffer);
    #endif
  }
}
