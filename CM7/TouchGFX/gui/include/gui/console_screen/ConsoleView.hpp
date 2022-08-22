#ifndef CONSOLEVIEW_HPP
#define CONSOLEVIEW_HPP

#include <gui_generated/console_screen/ConsoleViewBase.hpp>
#include <gui/console_screen/ConsolePresenter.hpp>

class ConsoleView : public ConsoleViewBase
{
public:
    ConsoleView();
    virtual ~ConsoleView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    void addMsg(const char* msg);
protected:
private:
    static constexpr uint8_t msgLinesCount = 10;
    uint16_t msgOffset = 0;
    uint8_t msgLine = 0;
};

#endif // CONSOLEVIEW_HPP
