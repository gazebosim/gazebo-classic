#include <stdio.h>

#include "gz.h"

using namespace libgazebo;

/// Delete a model by name
bool FactoryIface::DeleteEntity(const std::string &model_name)
{
  this->Lock(1);
  strcpy((char*)this->data->deleteEntity, model_name.c_str());
  this->Unlock();
  return true;
}
