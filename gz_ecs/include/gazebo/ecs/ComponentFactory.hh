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

#ifndef GAZEBO_ECS_COMPONENTFACTORY_HH_
#define GAZEBO_ECS_COMPONENTFACTORY_HH_

#include <map>
#include <functional>
#include <memory>

#include "gazebo/ecs/Component.hh"

#define GZ_COMPONENT_FACTORY_CREATE(type)\
    (gazebo::ecs::ComponentFactory::Create<type>(#type))

namespace gazebo
{
  namespace ecs
  {
    /// \brief A factor that registers and creates components.
    class ComponentFactory
    {
      /// \brief Register a Component type with a name
      /// \param[in] _name Name(key) of the type to register.
      /// \return True if the _name has not already been used.
      /// \sa Create
      public: template<typename T>
              static bool Register(const std::string &_name)
              {
                bool success = false;

                if (typeFactory.find(_name) == typeFactory.end())
                {
                  Component<T> tempForType;
                  types[_name] = tempForType.Type();

                  // Create a lambda function that will construct the given
                  // component type.
                  typeFactory[_name] = [] ()
                  {
                    // TODO using new bypasses idea that components are stored in adjacent memory
                    return new Component<T>();
                  };
                  success = true;
                }

                return success;
              }

      public: static ComponentType Type(const std::string &_name)
              {
                if (types.find(_name) != types.end())
                  return types[_name];
                else
                  return NO_COMPONENT;
              }

      /// \brief Create a new component.
      /// \param[in] _name Name (key) of the component type.
      /// \return A unique pointer to a new Component<T> object.
      /// \sa Register
      public: template<typename T>
              static ComponentPtr<T> Create(const std::string &_name)
              {
                if (typeFactory.find(_name) != typeFactory.end())
                {
                  return ComponentPtr<T>(
                      static_cast<Component<T>*>(typeFactory[_name]()));
                }
                else
                {
                  return nullptr;
                }
              }

      /// \brief Mapping of keys to construction functions.
      private: static std::map<std::string,
               std::function<ComponentBase *()>> typeFactory;

      private: static std::map<std::string, ComponentType> types;
    };
  }
}
#endif
