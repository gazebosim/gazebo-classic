/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_GUI_CONFIGWIDGET_PRIVATE_HH_
#define _GAZEBO_GUI_CONFIGWIDGET_PRIVATE_HH_

#include <map>
#include <string>

namespace google
{
  namespace protobuf
  {
    class Message;
  }
}

namespace gazebo
{
  namespace gui
  {
    class ConfigChildWidget;

    /// \class ConfigWidgetPrivate ConfigWidgetPrivate.hh
    /// \brief Private data for the ConfigWidget class.
    class ConfigWidgetPrivate
    {
      /// \brief A map of unique scoped names to correpsonding widgets.
      public: std::map <std::string, ConfigChildWidget *> configWidgets;

      /// \brief A copy of the message with fields to be configured by widgets.
      public: google::protobuf::Message *configMsg;
    };
  }
}
#endif
