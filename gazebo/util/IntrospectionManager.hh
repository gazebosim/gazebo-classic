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
#ifndef _GAZEBO_UTIL_INTROSPECTION_MANAGER_HH_
#define _GAZEBO_UTIL_INTROSPECTION_MANAGER_HH_

#include <functional>
#include <memory>
#include <string>
#include "gazebo/common/SingletonT.hh"
#include "gazebo/msgs/gz_string.pb.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace util
  {
    // Forward declare private data classes.
    class IntrospectionFilterPrivate;
    class IntrospectionManagerPrivate;

    /// \class IntrospectionFilter IntrospectionFilter.hh util/util.hh
    /// \brief
    class GZ_UTIL_VISIBLE IntrospectionFilter
    {
      /// \brief Constructor.
      public: IntrospectionFilter();

      /// \brief Destructor.
      public: virtual ~IntrospectionFilter() = default;

      /// \brief Assignment operator.
      /// \param[in] _other The new IntrospectionFilter.
      /// \return A reference to this instance.
      public: IntrospectionFilter &operator=(const IntrospectionFilter &_other);

      /// \brief Get List of items under observation by the filter.
      /// \return List of items.
      public: std::vector<std::string> Items() const;

      //public: std::string Topic() const;

      /// \brief Get List of items under observation by the filter.
      /// \return List of items.
      public: std::vector<std::string> &MutableItems();

      /// \brief Get a mutable reference to the message.
      /// \return A reference to the message.
      public: const msgs::Param_V &Msg() const;

      /// \brief Get a mutable reference to the message.
      /// \return A reference to the message.
      public: msgs::Param_V &MutableMsg();

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<IntrospectionFilterPrivate> dataPtr;
    };

    /// addtogroup gazebo_util
    /// \{

    /// \class IntrospectionManager IntrospectionManager.hh util/util.hh
    /// \brief
    class GZ_UTIL_VISIBLE IntrospectionManager
    {
      /// \brief ToDo.
      public: bool Register(const std::string &_item,
                            const std::string &_type,
                const std::function <bool (gazebo::msgs::GzString &_msg)> &_cb);

      /// \brief ToDo.
      public: bool Unregister(const std::string &_item);

      /// \brief ToDo.
      public: void SetFilter(const std::string &_topic,
                             const std::vector<std::string> _items);

      /// \brief ToDo.
      public: bool Filter(const std::string &_topic,
                          IntrospectionFilter &_filter) const;

      /// \brief ToDo.
      public: void RemoveFilter(const std::string &_topic);

      /// \brief Constructor.
      private: IntrospectionManager();

      /// \brief Destructor.
      private: virtual ~IntrospectionManager() = default;

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
