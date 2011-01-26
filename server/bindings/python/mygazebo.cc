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
#include <boost/python.hpp>
#include <python2.4/Python.h>
#include <string>

#include "Entity.hh"
#include "Model.hh"

using namespace boost::python;

BOOST_PYTHON_MODULE(mygazebo)
{
  class_<gazebo::Entity>("Entity", init<gazebo::Entity>())
  .def("GetId", &gazebo::Entity::GetId)
  .def("GetParentId", &gazebo::Entity::GetParentId)
  .add_property("parent", &gazebo::Entity::GetParent, &gazebo::Entity::SetParent)
  ;

  class_<gazebo::Model, bases<gazebo::Entity> >("Model")
  .def("GetName",&gazebo::Model::GetName, return_internal_reference<1>())
  .def("SetName",&gazebo::Model::SetName)
  ;

}


