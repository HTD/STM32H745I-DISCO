/**
 * @file		  console.c
 * @author	  CodeDog
 * @brief		  Simple text / debug console for TouchGFX projects.
 * @copyright	(c)2022 CodeDog, All rights reserved.
 */

#include "console.h"

uint8_t console_buffer[CONSOLE_BSIZE];
uint8_t console_update = 0;
static uint8_t X = 0;
static uint8_t Y = 0;

/**
 * @fn void scroll()
 * @brief Scrolls the console content 1 line up, adds an empty line at the bottom.
 */
static void scroll()
{
  memcpy(console_buffer, console_buffer + CONSOLE_LSIZE, CONSOLE_BSIZE - CONSOLE_LSIZE);
  memset(console_buffer + CONSOLE_BSIZE - CONSOLE_LSIZE, ' ', CONSOLE_WIDTH);
}

/**
 * @fn void line_break()
 * @brief Advances the cursor to the next line, optionally scrolls the console up if needed.
 */
static void line_break()
{
  X = 0;
  Y++;
  if (Y >= CONSOLE_HEIGHT)
  {
    scroll();
    Y--;
  }
  console_update = 1;
}

/**
 * @fn void console_clear()
 * @brief Clears the console buffer, moves the cursor to the first row of the first line.
 */
void console_clear()
{
  memset(console_buffer, ' ', CONSOLE_BSIZE);
  for (int i = 0; i < CONSOLE_HEIGHT; i++) console_buffer[CONSOLE_LSIZE * i + CONSOLE_WIDTH] = '\n';
  X = 0;
  Y = 0;
  console_update = 1;
}

/**
 * @fn void console_write(const char*)
 * @brief Writes the text to the console.
 * @param s Null terminated UTF-8 string to write.
 */
void console_write(const char* s)
{
  size_t l = strlen(s);
  size_t offset;
  for (int i = 0; i < l; i++)
  {
    if (s[i] == '\n')
    {
      line_break();
      continue;
    }
    offset = Y * CONSOLE_LSIZE + X;
    console_buffer[offset] = s[i];
    X++;
    if (X >= CONSOLE_WIDTH)
      line_break();
  }
  console_update = 1;
}

/**
 * @fn void console_writeln(const char*)
 * @brief Writes the text to the console and advances to the next line.
 * @param s Null terminated UTF-8 string to write.
 */
void console_writeln(const char* s)
{
  console_write(s);
  line_break();
}

/**
 * @fn void debug(const char*)
 * @brief Outputs a debug message.
 * @param s Null terminated UTF-8 string to write.
 */
void debug(const char* s)
{
  console_writeln(s);
}

/**
 * @fn void debug_i(const char*, int)
 * @brief Outputs a debug message with an integer value.
 * @param format sprintf type format.
 * @param i Integer to pass.
 */
void debug_i(const char* format, int i)
{
  uint8_t line[40];
  snprintf((char*)line, CONSOLE_WIDTH, format, i);
  console_writeln((char*)line);
}

/**
 * @fn void debug_f(const char*, float)
 * @brief Outputs a debug message with a float value.
 * @param format sprintf type format.
 * @param i Floating point value to pass.
 */
void debug_f(const char* format, float f)
{
  uint8_t line[40];
  snprintf((char*)line, CONSOLE_WIDTH, format, f);
  console_writeln((char*)line);
}

/**
 * @fn void debug_s(const char*, char*)
 * @brief Outputs a debug message with a string value.
 * @param format sprintf type format.
 * @param i Null terminated UTF-8 string to pass.
 */
void debug_s(const char* format, const char* s)
{
  uint8_t line[40];
  snprintf((char*)line, CONSOLE_WIDTH, format, s);
  console_writeln((char*)line);
}
