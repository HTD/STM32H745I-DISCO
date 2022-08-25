/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#ifndef CONSOLEVIEWBASE_HPP
#define CONSOLEVIEWBASE_HPP

#include <gui/common/FrontendApplication.hpp>
#include <mvp/View.hpp>
#include <gui/console_screen/ConsolePresenter.hpp>
#include <touchgfx/widgets/Box.hpp>
#include <touchgfx/widgets/TextArea.hpp>
#include <touchgfx/widgets/TextAreaWithWildcard.hpp>

class ConsoleViewBase : public touchgfx::View<ConsolePresenter>
{
public:
    ConsoleViewBase();
    virtual ~ConsoleViewBase() {}
    virtual void setupScreen();

protected:
    FrontendApplication& application() {
        return *static_cast<FrontendApplication*>(touchgfx::Application::getInstance());
    }

    /*
     * Member Declarations
     */
    touchgfx::Box __background;
    touchgfx::TextArea title;
    touchgfx::TextAreaWithOneWildcard console;

    /*
     * Wildcard Buffers
     */
    static const uint16_t CONSOLE_SIZE = 410;
    touchgfx::Unicode::UnicodeChar consoleBuffer[CONSOLE_SIZE];

private:

};

#endif // CONSOLEVIEWBASE_HPP
