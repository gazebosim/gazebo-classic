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
#ifndef _GAZEBO_SPLASH_SCREEN_HH_
#define _GAZEBO_SPLASH_SCREEN_HH_

#include <string>

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class SplashScreenPrivate;

    /// \brief Splash screen that displays an icon and a message.
    class GZ_GUI_VISIBLE SplashScreen : public QObject
    {
      Q_OBJECT

      /// \brief Constructor
      public: SplashScreen();

      /// \brief Destructor
      public: virtual ~SplashScreen();

      /// \brief Returns whether the splash screen is visible.
      /// \return True if the splash screen is visible, false otherwise.
      public: bool Visible() const;

      /// \brief Qt callback to update the splash screen
      private slots: void Update();

      /// \internal
      /// \brief Pointer to private data.
      private: SplashScreenPrivate *dataPtr;
    };
  }
}

#endif
