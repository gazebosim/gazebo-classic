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
#include "gazebo/msgs/any.pb.h"
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
      /// \brief ToDo.
      public: bool Register(const std::string &_item,
                            const std::string &_type,
                            const std::function <bool(
                                gazebo::msgs::Any &_msg)> &_cb);

      /// \brief ToDo.
      public: bool Unregister(const std::string &_item);

      /// \brief ToDo.
      public: void SetFilter(const std::string &_filterId,
                             const std::vector<std::string> &_items);

      /// \brief ToDo.
      public: bool Filter(const std::string &_filterId,
                          std::vector<std::string> &_items) const;

      /// \brief ToDo.
      public: bool RemoveFilter(const std::string &_filterId);

      /// \brief ToDo.
      public: void Update();

      /// \brief Constructor.
      private: IntrospectionManager();

      /// \brief Destructor.
      private: virtual ~IntrospectionManager();

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
