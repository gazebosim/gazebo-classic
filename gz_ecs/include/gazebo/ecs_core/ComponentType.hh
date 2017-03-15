/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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


#ifndef GAZEBO_ECS_CORE_COMPONENTTYPE_HH_
#define GAZEBO_ECS_CORE_COMPONENTTYPE_HH_

#include <typeinfo>
#include <type_traits>

namespace gazebo
{
namespace ecs_core
{

/// \brief A type with a code identifying the type of component
typedef std::size_t ComponentType;

/// \brief a for getting a type hash code for identifying a component
template <typename T>
ComponentType GetComponentType()
{
  // TODO is POD too strict here? The real problem is EntityManager may
  // not be able to guarantee calling the destructor of a componenent.
  static_assert(std::is_pod<T>::value == true, "Component must be Plain Old Data");
  return typeid(T).hash_code();
}

}
}

#endif

