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
#ifndef GAZEBO_UTIL_INTROSPECTION_MANAGER_HH_
#define GAZEBO_UTIL_INTROSPECTION_MANAGER_HH_

#include <functional>
#include <memory>
#include <set>
#include <string>
#include "gazebo/common/Console.hh"
#include "gazebo/common/SingletonT.hh"
#include "gazebo/msgs/any.pb.h"
#include "gazebo/msgs/empty.pb.h"
#include "gazebo/msgs/gz_string.pb.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/msgs/param.pb.h"
#include "gazebo/msgs/param_v.pb.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace util
  {
    // Forward declare private data classes.
    class IntrospectionManagerPrivate;

    /// addtogroup gazebo_util
    /// \{

    /// \class IntrospectionManager IntrospectionManager.hh util/util.hh
    /// \brief
    class GZ_UTIL_VISIBLE IntrospectionManager
      : public SingletonT<IntrospectionManager>
    {
      /// \brief Get the unique ID of this manager.
      /// \return Manager ID.
      public: std::string Id() const;

      /// \brief Register a new item in the introspection manager.
      /// \param[in] _item New item. E.g.: /default/world/model1/pose
      /// \param[in] _cb Callback used to get the last update for this item.
      /// \result True when the registration succeed or false otherwise
      /// (item already existing).
      public: template<typename T>
      bool Register(const std::string &_item,
                    const std::function<T()> &_cb)
      {
        auto func = [=]()
        {
          return msgs::ConvertAny(_cb());
        };

        return this->Register(_item, func);
      }

      /// \brief Unregister an existing item from the introspection manager.
      /// \param[in] _item Item to remove.
      /// \return True if the unregistration succeed or false otherwise
      /// (the item was not previously registered).
      public: bool Unregister(const std::string &_item);

      /// \brief Unregister all items.
      public: void Clear();

      /// \brief Get a copy of the items already registered in this manager.
      /// \return Set of registered items.
      public: std::set<std::string> Items() const;

      /// \brief Update all the items under observation and publish updates
      /// through all the topics. The message received in the update will
      /// contain the name and latest values of all the items specified
      /// in the filter.
      /// If there are changes in the items list since the last update,
      /// a new message is published under the topic
      /// "/introspection/<manager_id>/items_update".
      public: void Update();

      /// \brief If there are changes in the items list since the last update,
      /// a new message is published under the topic
      /// "/introspection/<manager_id>/items_update".
      public: void NotifyUpdates();

      /// \brief Constructor.
      private: IntrospectionManager();

      /// \brief Destructor.
      private: virtual ~IntrospectionManager();

      /// \brief Register a new item in the introspection manager.
      /// \param[in] _item New item. E.g.: /default/world/model1/pose
      /// \param[in] _cb Callback used to get the last update for this item.
      /// \result True when the registration succeed or false otherwise
      /// (item already existing).
      private: bool Register(const std::string &_item,
                             const std::function <gazebo::msgs::Any()> &_cb);

      /// \brief Create a new filter for observing item updates. This function
      /// will create a new topic for sending periodic updates of the items
      /// specified in the filter.
      /// \param[in] _newItems Non-empty set of items to observe.
      /// \param[out] _filterId Unique ID of the filter. You'll need this ID
      /// for future filter updates or for removing it. After the filter
      /// creation, a client should subscribe to the topic
      /// /introspection/filter/<filter_id> for receiving updates.
      /// \return True if the filter was successfully created or false otherwise
      private: bool NewFilter(const std::set<std::string> &_newItems,
                              std::string &_filterId);

      /// \brief Update an existing filter with a different set of items.
      /// \param[in] _filterId ID of the filter to update.
      /// \param[in] _newItems Non-empty set of items to be observed.
      /// \return True if the filter was successfuly updated or false otherwise.
      private: bool UpdateFilter(const std::string &_filterId,
                                 const std::set<std::string> &_newItems);

      /// \brief Remove an existing filter.
      /// \param[in] _filterId ID of the filter to remove.
      /// \return True if the filter was successfully removed or false otherwise
      private: bool RemoveFilter(const std::string &_filterId);

      /// \brief Internal callback for creating a filter via service request.
      /// \param[in] _req Input parameter of the service request. The service
      /// expects a collection of one or more parameters with name "item" and a
      /// value of type STRING containing the name of the item to observe.
      /// \param[out] _rep Output parameter of the service request. It contains
      /// the filter ID created.
      /// \param[out] _result True when the operation succeed or false
      ///  otherwise. _rep should be ignored when _result is false.
      private: void NewFilter(const gazebo::msgs::Param_V &_req,
                              gazebo::msgs::GzString &_rep,
                              bool &_result);

      /// \brief Internal callback for updating a filter via service request.
      /// \param[in] _req Input parameter of the service request. The service
      /// expects one parameter with name "filter_id", value type STRING and
      /// containing the filter ID to be updated. Also, it's expected to have
      /// a collection of one or more parameters with name "item" and a
      /// value of type STRING containing the name of the item to observe.
      /// \param[out] _rep Not used.
      /// \param[out] _result True when the filter was successfully updated or
      /// false otherwise.
      private: void UpdateFilter(const gazebo::msgs::Param_V &_req,
                                 gazebo::msgs::Empty &_rep,
                                 bool &_result);

      /// \brief Internal callback for removing a filter via service request.
      /// \param[in] _req Input parameter of the service request. The service
      /// expects exactly one parameter with name "filter_id", value type STRING
      /// and containing the filter ID to be removed.
      /// \param[out] _rep Not used.
      /// \param[out] _result True when the filter was successfully removed or
      /// false otherwise.
      private: void RemoveFilter(const gazebo::msgs::Param_V &_req,
                                 gazebo::msgs::Empty &_rep,
                                 bool &_result);

      /// \brief Internal callback for listing all the items registered in the
      /// introspection manager via service request.
      /// \param[in] _req Not used.
      /// \param[out] _rep Collection of parameters representing the items
      /// registered. Each parameter should have a name "item", followed by a
      /// value of type STRING.
      /// \param[out] _result True when the request succeeded.
      private: void Items(const gazebo::msgs::Empty &_req,
                          gazebo::msgs::Param_V &_rep,
                          bool &_result);

      /// \brief Helper function for creating a random string identifier.
      /// E.g.: "abcbgh", "egyufd".
      /// \param[in] _size Length of the identifier in chars.
      /// \return The random ID.
      private: std::string CreateRandomId(const unsigned int &_size) const;

      /// \brief Helper function for validating messages.
      /// \param[in] _msg Message to be validated.
      /// \param[in] _allowedValues Set of valid parameter names allowed in the
      /// message.
      /// \return True when the message contains only allowed parameter names,
      /// its values are STRING and the values exist.
      private: bool ValidateParameter(const gazebo::msgs::Param &_msg,
                             const std::set<std::string> &_allowedValues) const;

      /// \brief This is a singleton.
      private: friend class SingletonT<IntrospectionManager>;

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<IntrospectionManagerPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
