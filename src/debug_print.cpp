#include "debug_print.hpp"

const char *debugLevelLabels[] = {
    "",
    "[ERROR] ",
    "[WARN] ",
    "[INFO] ",
    "[DETAIL] "};

static DebugLevel currentDebugLevel = DEBUG_WARN;

void setDebugLevel(DebugLevel level) {
  currentDebugLevel = level;
}

DebugLevel getDebugLevel() {
  return currentDebugLevel;
}

void debugPrint(DebugLevel level, const char *format, ...) {
  if(level <= currentDebugLevel) {
    Serial.print(debugLevelLabels[level]);

    va_list args;
    va_start(args, format);
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    Serial.println(buffer);
  }
}