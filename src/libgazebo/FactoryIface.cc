/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
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
