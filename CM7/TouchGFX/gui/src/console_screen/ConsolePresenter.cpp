#include <gui/console_screen/ConsoleView.hpp>
#include <gui/console_screen/ConsolePresenter.hpp>

ConsolePresenter::ConsolePresenter(ConsoleView& v)
    : view(v)
{

}

void ConsolePresenter::activate()
{
    model->isMessagePresenterReady = true;
}

void ConsolePresenter::deactivate()
{
    model->isMessagePresenterReady = false;
}

/**
 * @brief Adds a message to the console. When maximum number lines displayed, the console will be clear.
 * @param msg A message to add.
 */
void ConsolePresenter::addMsg(const char* msg)
{
    view.addMsg(msg);
}
