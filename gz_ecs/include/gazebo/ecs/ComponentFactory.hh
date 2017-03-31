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

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <new>
#include <typeinfo>
#include <vector>



namespace gazebo
{
  namespace ecs
  {

  /// \brief ID to refer to a component type
  typedef int ComponentType;

  /// \brief Special value returned to say there is no component
  static const ComponentType NO_COMPONENT = -1;

  /// \brief forward declaration for friendship
  class ComponentFactory;


  /// \brief Holds information about a component type
  class ComponentTypeInfo
  {
    /// \brief Constructs without allocating memory
    public: std::function<void (void*)> constructor;

    /// \brief Destructs component without freeing memory
    public: std::function<void (void*)> destructor;

    /// \brief Moves component from one memory location to another
    public: std::function<void (void*, void*)> mover;

    /// \brief Size of an instantiated component in bytes
    public: std::size_t size;

    /// \brief Name of the component type
    public: std::string name;

    friend ComponentFactory;

    private: template <typename T>
            static ComponentTypeInfo From()
            {
              ComponentTypeInfo info;
              info.constructor = [](void *_location)
              {
                // placement new operator, doesn't allocate memory
                new (_location) T();
              };

              info.destructor = [](void *_location)
              {
                // explicit call to destructor without freeing memory
                static_cast<T*>(_location)->~T();
              };

              // Store size so space can be allocated elsewhere
              info.size = sizeof(T);

              info.mover = [](void const *_from, void *_to)
              {
                // Move component from one location to another
                // Making a lambda for this allows the compiler to unroll
                // the loop completely, which probably makes no difference
                // because components will probably be small (< 32 bytes)
                for (int i = 0; i < sizeof(T); ++i)
                {
                  static_cast<char*>(_to)[i] =
                    static_cast<char const*>(_from)[i];
                }
              };
              return info;
            }
  };


    /// \brief A factor that registers and creates components.
    class ComponentFactory
    {
      /// \brief Register a Component type with a name
      /// \param[in] _name Name(key) of the type to register.
      /// \return True if the _name has not already been used.
      /// \sa Create
      public: template <typename T>
              static bool Register(const std::string &_name)
              {
                bool success = false;
                const std::size_t hash = typeid(T).hash_code();

                std::lock_guard<std::mutex> lock(mtx);
                if (typesByName.find(_name) == typesByName.end()
                    && typesByHash.find(hash) == typesByHash.end())
                {
                  ComponentTypeInfo info = ComponentTypeInfo::From<T>();

                  // Store name for debugging
                  info.name = _name;

                  ComponentType id = typeInfoById.size();
                  typesByName[_name] = id;
                  typesByHash[hash] = id;
                  typeInfoById.push_back(info);
                  success = true;
                }
                return success;
              }

      /// \brief Return a ComponentType for a component by Name
      /// TODO This isn't templated or inlined, it could be in a source file
      public: static ComponentType Type(const std::string &_name)
              {
                std::lock_guard<std::mutex> lock(mtx);
                if (typesByName.find(_name) != typesByName.end())
                  return typesByName[_name];
                return NO_COMPONENT;
              }

      /// \brief Return a ComponentType for a component by actual type
      public: template <typename T>
              static ComponentType Type()
              {
                std::lock_guard<std::mutex> lock(mtx);
                std::size_t hash = typeid(T).hash_code();
                if (typesByHash.find(hash) != typesByHash.end())
                  return typesByHash[hash];
              }

      /// \brief Get the full type information
      /// TODO This isn't templated or inlined, it could be in a source file
      public: static ComponentTypeInfo const & TypeInfo(ComponentType _type)
              {
                std::lock_guard<std::mutex> lock(mtx);
                if (_type >= 0 && _type < typeInfoById.size())
                  return typeInfoById[_type];
              }

      /// \brief Lock for thread safety
      public: static std::mutex mtx;

      /// \brief Mapping of names to component type
      private: static std::map<std::string, ComponentType> typesByName;

      /// \brief Mapping of type hash to component type
      private: static std::map<std::size_t, ComponentType> typesByHash;

      /// \brief Map ComponentType to type information
      ///
      /// index is the same as ComponentType
      private: static std::vector<ComponentTypeInfo> typeInfoById;
    };
  }
}

#endif
