/**
 * @file      console.c
 * @author    CodeDog
 * @brief     Simple text / debug console for TouchGFX projects, header file.
 * @copyright (c)2022 CodeDog, All rights reserved.
 */

#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CONSOLE_WIDTH 40 ///< Console width in characters.
#define CONSOLE_HEIGHT 10 ///< Console height in characters.
#define CONSOLE_LSIZE (CONSOLE_WIDTH + 1) ///< Console line size including LF at the end.
#define CONSOLE_BSIZE (CONSOLE_WIDTH * CONSOLE_HEIGHT + CONSOLE_HEIGHT) ///< Console buffer size.

extern uint8_t console_buffer[CONSOLE_BSIZE]; ///< The buffor for console text.
extern uint8_t console_update; ///< 0: no changes, 1: updated.

#ifdef __cplusplus
extern "C"
{
#endif

void console_clear();
void console_write(const char* s);
void console_writeln(const char* s);

void debug(const char* s);
void debug_i(const char* format, int i);
void debug_f(const char* format, float f);
void debug_s(const char* format, const char* s);

#ifdef __cplusplus
}
#endif

