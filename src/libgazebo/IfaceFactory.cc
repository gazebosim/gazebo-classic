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
/*
 * Desc: Factory for creating iface
 * Author: Nate Koenig
 * Date: 06 May 2007
 * SVN info: $Id$
 */

#include <sstream>
#include <iostream>

#include "gz.h"
#include "IfaceFactory.hh"

using namespace libgazebo;

std::map<std::string, IfaceFactoryFn> IfaceFactory::ifaces;

// Register a iface class.  Use by dynamically loaded modules
void IfaceFactory::RegisterIface(std::string classname, IfaceFactoryFn factoryfn)
{
  ifaces[classname] = factoryfn;
}

// Create a new instance of a iface.  Used by the world when reading
// the world file.
Iface *IfaceFactory::NewIface(const std::string &classname)
{
  if (ifaces.find(classname) != ifaces.end())
  {
    return (ifaces[classname]) ();
  }
  else
  {
    std::ostringstream stream;
    stream << "Unable to make interface of type " << classname;
    throw(stream.str());
  }


  return NULL;
}
