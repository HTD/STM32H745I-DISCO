#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#ifndef SIMULATOR
#include "main.h"
#endif

Model::Model() : modelListener(0)
{

}

void Model::tick()
{
#ifndef SIMULATOR
  if (console_update && isMessagePresenterReady)
  {
      modelListener->updateConsole();
      console_update = 0;
  }
#endif
}
