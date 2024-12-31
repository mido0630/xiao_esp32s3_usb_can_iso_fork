#ifndef DEBUG_PRINT_HPP
#define DEBUG_PRINT_HPP

#include <Arduino.h>
#include <stdarg.h>

enum DebugLevel {
  DEBUG_NONE = 0, // デバッグ出力なし
  DEBUG_ERROR,    // エラーのみ
  DEBUG_WARN,     // エラーと警告
  DEBUG_INFO,     // 概要
  DEBUG_DETAIL    // 詳細
};

extern const char *debugLevelLabels[];

void       setDebugLevel(DebugLevel level);
DebugLevel getDebugLevel();
void       debugPrint(DebugLevel level, const char *format, ...);

#endif // DEBUG_PRINT_HPP