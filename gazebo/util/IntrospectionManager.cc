/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <functional>
#include <string>
#include "gazebo/msgs/gz_string.pb.h"
#include "gazebo/util/IntrospectionManagerPrivate.hh"
#include "gazebo/util/IntrospectionManager.hh"

using namespace gazebo;
using namespace util;

//////////////////////////////////////////////////
IntrospectionManager::IntrospectionManager()
: dataPtr(new IntrospectionManagerPrivate)
{

}

//////////////////////////////////////////////////
IntrospectionManager::~IntrospectionManager()
{

}

//////////////////////////////////////////////////
bool IntrospectionManager::Register(const std::string &_item,
    const std::function <bool (gazebo::msgs::GzString &_msg)> &_cb)
{
  return true;
}

//////////////////////////////////////////////////
bool IntrospectionManager::Unregister(const std::string &_item)
{
  return true;
}
