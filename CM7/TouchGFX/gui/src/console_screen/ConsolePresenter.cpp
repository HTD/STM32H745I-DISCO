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
 * @brief Updates the console view.
 */
void ConsolePresenter::updateConsole()
{
    view.update();
}
