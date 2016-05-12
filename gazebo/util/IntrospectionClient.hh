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
#ifndef GAZEBO_UTIL_INTROSPECTION_CLIENT_HH_
#define GAZEBO_UTIL_INTROSPECTION_CLIENT_HH_

#include <chrono>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace util
  {
    // Forward declare private data classes.
    class IntrospectionClientPrivate;

    /// addtogroup gazebo_util
    /// \{

    /// \class IntrospectionClient IntrospectionClient.hh util/util.hh
    /// \brief
    class GZ_UTIL_VISIBLE IntrospectionClient
    {
      /// \brief Constructor.
      public: IntrospectionClient();

      /// \brief Destructor.
      public: virtual ~IntrospectionClient();

      /// \brief Wait for introspection managers to appear on the network.
      /// This function is useful for clients that want to pause until
      /// one or more IntrospectionManagers becomes available (such as
      /// when gzserver is run).
      /// \param[in] _timeOut Maximum duration to wait. A value of
      /// std::chrono::duration::zero indicates that the function should
      /// wait indefinitely. An indefinite wait is the default behavior.
      /// \return List of available managers.
      /// \sa Managers()
      public: std::set<std::string> WaitForManagers(
                  const std::chrono::milliseconds _timeOut =
                  std::chrono::milliseconds::zero()) const;

      /// \brief Get the list of introspection managers currently available.
      /// \return Set of unique manager IDs.
      public: std::set<std::string> Managers() const;

      /// \brief Create a new filter for observing item updates. This function
      /// will create a new topic for sending periodic updates of the items
      /// specified in the filter. This function will block until the result is
      /// received.
      /// \param[in] _managerID ID of the manager to request the operation.
      /// \param[in] _newItems Non-empty set of items to observe.
      /// \param[out] _filterId Unique ID of the filter. You'll need this ID
      /// for future filter updates or for removing it.
      /// \param[out] _newTopic After the filter creation, a client should
      /// subscribe to this topic for receiving updates.
      /// \return True if the filter was successfully created or false otherwise
      public: bool NewFilter(const std::string &_managerId,
                             const std::set<std::string> &_newItems,
                             std::string &_filterId,
                             std::string &_newTopic) const;

      /// \brief Create a new filter for observing item updates. This function
      /// will create a new topic for sending periodic updates of the items
      /// specified in the filter. This function will not block, the result
      /// will be received in a callback function.
      /// \param[in] _managerID ID of the manager to request the operation.
      /// \param[in] _newItems Non-empty set of items to observe.
      /// \param[in] _cb Callback function executed when the response arrives.
      /// The callback has the following parameters:
      ///   \param[in] _filterId Unique ID of the filter. You'll need this ID
      /// for future filter updates or for removing it.
      ///   \param[in] _newTopic After the filter creation, a client should
      /// subscribe to this topic for receiving updates.
      ///   \param[in] _result Result of the request. If false, there was
      /// a problem executing your request.
      /// \return True if the request was successfully sent.
      public: bool NewFilter(const std::string &_managerId,
                             const std::set< std::string> &_newItems,
                             const std::function <void(
                                 const std::string &_filterId,
                                 const std::string &_newTopic,
                                 const bool _result)> &_cb) const;

      /// \brief Update an existing filter with a different set of items.
      /// This function will block until the result is received.
      /// \param[in] _managerID ID of the manager to request the operation.
      /// \param[in] _filterId ID of the filter to update.
      /// \param[in] _newItems Non-empty set of items to be observed.
      /// \return True if the filter was successfuly updated or false otherwise.
      public: bool UpdateFilter(const std::string &_managerId,
                                const std::string &_filterId,
                                const std::set<std::string> &_newItems) const;

      /// \brief Update an existing filter with a different set of items.
      /// This function will not block, the result will be received in a
      /// callback function.
      /// \param[in] _managerID ID of the manager to request the operation.
      /// \param[in] _filterId ID of the filter to update.
      /// \param[in] _newItems Non-empty set of items to be observed.
      /// \param[in] _cb Callback function executed when the response arrives.
      /// The callback has the following parameter:
      ///   \param[in] _result Result of the request. If false, there was
      /// a problem executing your request.
      /// \return True if the request was successfully sent.
      public: bool UpdateFilter(const std::string &_managerId,
                                const std::string &_filterId,
                                const std::set<std::string> &_newItems,
                                const std::function <void(
                                    const bool _result)> &_cb) const;

      /// \brief Remove all existing filters.
      /// This function will block until the result is received.
      /// \return True if the filters were successfully removed
      /// or false otherwise
      public: bool RemoveAllFilters() const;

      /// \brief Remove an existing filter.
      /// This function will block until the result is received.
      /// \param[in] _managerID ID of the manager to request the operation.
      /// \param[in] _filterId ID of the filter to remove.
      /// \return True if the filter was successfully removed or false otherwise
      public: bool RemoveFilter(const std::string &_managerId,
                                const std::string &_filterId) const;

      /// \brief Remove an existing filter.
      /// This function will not block, the result will be received in a
      /// callback function.
      /// \param[in] _managerID ID of the manager to request the operation.
      /// \param[in] _filterId ID of the filter to remove.
      /// \param[in] _cb Callback function executed when the response arrives.
      /// The callback has the following parameter:
      ///   \param[in] _result Result of the request. If false, there was
      /// a problem executing your request.
      /// \return True if the request was successfully sent.
      public: bool RemoveFilter(const std::string &_managerId,
                                const std::string &_filterId,
                                const std::function <void(
                                    const bool _result)> &_cb) const;

      /// \brief Get a copy of the items already registered.
      /// This function will block until the result is received.
      /// \param[in] _managerID ID of the manager to request the operation.
      /// \param[out] _items The list of items.
      /// \return True if the operation succeed or false otherwise
      /// (wrong managerID, transport problem).
      public: bool Items(const std::string &_managerId,
                         std::set<std::string> &_items) const;

      /// \brief Get a copy of the items already registered.
      /// This function will not block, the result will be received in a
      /// callback function.
      /// \param[in] _managerID ID of the manager to request the operation.
      /// \param[in] _cb Callback function executed when the response arrives.
      /// The callback has the following parameter:
      ///   \param[in] _items The list of items.
      ///   \param[in] _result Result of the request. If false, there was
      /// a problem executing your request.
      /// \return True if the request was successfully sent.
      public: bool Items(const std::string &_managerId,
                         const std::function <void(
                             const std::set<std::string> &_items,
                             const bool _result)> &_cb) const;

      /// \brief Check if the _item is registered on a manager with
      /// _managerId.
      /// \param[in] _managerId Id of the manager to query.
      /// \param[in] _item Item name for the query.
      /// \return True if the introspection manager with ID==_managerId has
      /// an item registered with name == _item.
      public: bool IsRegistered(const std::string &_managerId,
                                const std::string &_item) const;

      /// \brief Check if the _items are registered on a manager with
      /// _managerId.
      /// \param[in] _managerId Id of the manager to query.
      /// \param[in] _items Set of item names for the query.
      /// \return True if the introspection manager with ID==_managerId has
      /// all the items registered.
      public: bool IsRegistered(const std::string &_managerId,
                                const std::set<std::string> &_items) const;

      /// \brief Check if there are any current filters using a manager with
      /// _managerId.
      /// \param[in] _managerId Id of the manager to query.
      /// return True if the instrospection manager with ID==_managerId is
      /// being used in some of the current filters.
      private: bool IsManagerUsed(const std::string &_managerId) const;

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<IntrospectionClientPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
