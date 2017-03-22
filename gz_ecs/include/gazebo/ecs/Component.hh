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

#ifndef GAZEBO_ECS_COMPONENT_HH_
#define GAZEBO_ECS_COMPONENT_HH_

#include <memory>

namespace gazebo
{
  namespace ecs
  {
    typedef int ComponentId;
    typedef int ComponentType;

    static const ComponentType NO_COMPONENT = -1;

    /// \brief Component base class.
    /// Do not use this class directly.
    class ComponentBase
    {
      public: ComponentBase() : id(idCounter++)
              {
              }

      public: virtual ~ComponentBase() {}

      /// \brief Get the unique ID for this component type.
      public: virtual ComponentType Type() const = 0;

      public: ComponentId Id() const
              {
                return this->id;
              }

      /// \brief Unique id for this component.
      private: ComponentId id;

      /// \brief Counter used to generate ids.
      private: static ComponentId idCounter;

      /// \brief Counter used to create a unique types.
      private: static ComponentType typeCounter;
    };

    /// \brief A component class contains data, and has a unique ID.
    template <typename T>
    class Component : public ComponentBase
    {
      public: Component() : ComponentBase()
              {
                if (type == -1)
                  type = typeCounter++;
              }

      public: virtual ~Component() {}

      /// \brief Get the unique ID for this component type.
      public: virtual ComponentType Type() const
              {
                return this->type;
              }

      public: T data;

      /// \brief An id for this component type
      private: static ComponentType type;
    };

    // Create a unique_ptr template alias for ease of use.
    template<typename T>
    using ComponentPtr = std::unique_ptr<Component<T>>;

    template<typename T>
    ComponentId gazebo::ecs::Component<T>::type = -1;
  }
}
#endif
