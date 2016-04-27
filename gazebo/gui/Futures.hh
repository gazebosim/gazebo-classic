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
#ifndef GAZEBO_GUI_FUTURES_HH_
#define GAZEBO_GUI_FUTURES_HH_

#include <future>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \brief The Futures class holds static std::future variables. Each
    /// variable is used to perform some asynchronous operation on start.
    ///
    /// Example usage of an existing future:
    ///
    /// \code{.cpp}
    /// if (Futures::introspectionClientFuture.valid())
    /// {
    ///   Futures::introspectionClientFuture.get();
    /// }
    /// \endcode
    ///
    /// To add a new future:
    ///
    /// 1. Add a new static public std::future in Futures.hh
    /// 2. Declare the static member at the top of Futures.cc
    /// 3. Instantiate the future in the Future's constructor.
    class GZ_GUI_VISIBLE Futures
    {
      /// \brief Private constructor. No one needs to instantiate this
      /// class. Just use the public static member variables.
      private: Futures();

      /// \brief Private destructor. No one needs to instantiate this
      /// class. Just use the public static member variables.
      private: ~Futures();

      /// \brief This will get the introspection managers, used by plotting.
      public: static std::future<void> introspectionClientFuture;
    };

    /// \}
  }
}
#endif
