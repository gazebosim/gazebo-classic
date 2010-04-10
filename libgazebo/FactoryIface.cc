#include <stdio.h>

#include "gz.h"

using namespace gazebo;

/// \brief Delete a model by name
bool FactoryIface::DeleteModel(const std::string &model_name)
{
  this->Lock(1);
  strcpy((char*)this->data->deleteModel, model_name.c_str());
  this->Unlock();
  return true;
}
