#include <gui/console_screen/ConsoleView.hpp>
#include <string.h>
#include <stdio.h>
#include "console.h"

ConsoleView::ConsoleView()
{

}

void ConsoleView::setupScreen()
{
  ConsoleViewBase::setupScreen();
}

void ConsoleView::tearDownScreen()
{
  ConsoleViewBase::tearDownScreen();
}

/**
 * @brief Updates the console view.
 */
void ConsoleView::update()
{
  Unicode::fromUTF8(console_buffer, consoleBuffer, CONSOLE_SIZE);
  console.invalidate();
}
