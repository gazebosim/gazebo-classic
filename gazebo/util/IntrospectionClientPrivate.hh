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
#ifndef GAZEBO_UTIL_INTROSPECTION_CLIENT_PRIVATE_HH_
#define GAZEBO_UTIL_INTROSPECTION_CLIENT_PRIVATE_HH_

#include <map>
#include <mutex>
#include <string>
#include <ignition/transport.hh>

namespace gazebo
{
  namespace util
  {
    /// \brief Private data for the IntrospectionClient class.
    class IntrospectionClientPrivate
    {
      /// \brief Node used for communications.
      public: ignition::transport::Node node;

      /// \brief Timeout (ms) used for service requests.
      public: const unsigned int kTimeout = 500;

      /// \brief Store all the active filters in this client.
      /// The key is the filter ID.
      /// The value is the manager ID where the filter is located.
      public: std::map<std::string, std::string> filters;

      /// \brief A mutex to guarantee mutual exclusion.
      public: std::mutex mutex;
    };
  }
}
#endif
