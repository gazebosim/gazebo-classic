/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_UTIL_INTROSPECTION_MANAGER_PRIVATE_HH_
#define GAZEBO_UTIL_INTROSPECTION_MANAGER_PRIVATE_HH_

#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <ignition/transport.hh>
#include "gazebo/msgs/any.pb.h"
#include "gazebo/msgs/param_v.pb.h"
#include "gazebo/util/IntrospectionManager.hh"

namespace gazebo
{
  namespace util
  {
    /// \brief Private data for the IntrospectionFilter class.
    struct IntrospectionFilter
    {
      /// \brief Items observed by this filter.
      std::set<std::string> items;

      /// \brief Message containing the next update. A message is a collection
      /// of items and values.
      msgs::Param_V msg;
    };

    /// \brief Todo.
    struct ObservedItem
    {
      /// \brief ToDo.
      gazebo::msgs::Any lastValue;

      /// \brief ToDo.
      std::set<std::string> filters;
    };

    /// \brief Private data for the IntrospectionManager class.
    class IntrospectionManagerPrivate
    {
      /// \brief List of active filters.
      /// The key is the topic where the filter publishes updates.
      /// The value is the associated introspection filter.
      public: std::map<std::string, IntrospectionFilter> filters;

      /// \brief List of all registered items.
      /// The key contains the item name.
      /// The value contains the string representation of the protobuf type
      /// that stores the value.
      /// E.g.: allItems["model1::pose"] = "gazebo::msgs::Pose"
      public: std::map<std::string, std::function <gazebo::msgs::Any ()>>
          allItems;

      /// \brief List of items that have at least one active observer.
      /// The key contains the item name.
      /// The value contains the last value stored for this item, as well as a
      /// list of all the filters that contain the item.
      public: std::map<std::string, ObservedItem> observedItems;

      /// \brief Mutex to make this class thread-safe.
      public: mutable std::mutex mutex;

      /// \brief Node used for communications.
      public: ignition::transport::Node node;

      /// \brief ID of this manager.
      public: std::string managerId;

      /// \brief Prefix used for announcing services.
      /// E.g."/introspection/abcxyz/".
      public: std::string prefix;

      /// \brief Flag that will be true when the list of registered items has
      /// changed since the last update.
      public: bool itemsUpdated = false;
    };
  }
}
#endif
