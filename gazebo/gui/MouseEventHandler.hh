/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef _MOUSE_EVENT_HANDLER_HH_
#define _MOUSE_EVENT_HANDLER_HH_

#include <boost/function.hpp>
#include <string>
#include <list>

#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/MouseEvent.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Processes and filters mouse events.
    class MouseEventHandler : public SingletonT<MouseEventHandler>
    {
      public: typedef boost::function<bool (const common::MouseEvent &_event)>
              MouseEventFilter;

      /// \brief Constructor
      private: MouseEventHandler();

      /// \brief Destructor
      private: virtual ~MouseEventHandler();

      public: void AddFilter(const std::string &_name,
                  MouseEventFilter _filter);

      public: void Handle(const common::MouseEvent &_event);

      private: std::list<MouseEventFilter> filters;

      private: friend class SingletonT<MouseEventHandler>;
    };
  }
}
#endif
