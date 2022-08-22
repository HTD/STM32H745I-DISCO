#include <gui/console_screen/ConsoleView.hpp>
#include <string.h>

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
 * @brief Adds a message to the console. When maximum number lines displayed, the console will be clear.
 * @param msg A message to add.
 */
void ConsoleView::addMsg(const char* msg)
{
    if (msgLine >= msgLinesCount) {
        msgOffset = 0;
        msgLine = 0;
    }
    Unicode::fromUTF8(reinterpret_cast<uint8_t*>(const_cast<char*>(msg)), consoleBuffer + msgOffset, CONSOLE_SIZE - msgOffset);
    msgOffset += strlen(msg);
    Unicode::fromUTF8(reinterpret_cast<uint8_t*>(const_cast<char*>("\n")), consoleBuffer + msgOffset, CONSOLE_SIZE - msgOffset);
    msgOffset++;
    msgLine++;
    console.invalidate();
}
