/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SPACENAV_HH_
#define _GAZEBO_SPACENAV_HH_

namespace gazebo
{
  namespace gui
  {
    class SpaceNavPrivate;

    /// \class SpaceNav SpacNav.hh gui/gui.hh
    /// \brief Interface to the space navigator joystick.
    ///
    /// The space nav can be configured in ~/.gazebo/gui.in with the following:
    ///
    /// \code
    ///   [spacenav]
    ///   deadband_x = 0.1
    ///   deadband_y = 0.1
    ///   deadband_z = 0.1
    ///   deadband_rx = 0.1
    ///   deadband_ry = 0.1
    ///   deadband_rz = 0.1
    ///   topic=~/spacenav/joy
    /// \endcode
    ///
    ///  - deadband is the threshold below which a zero value is returned
    ///      - deadband_x : Deadband for x translation
    ///      - deadband_y : Deadband for y translation
    ///      - deadband_z : Deadband for z translation
    ///      - deadband_rx : Deadband for x-axis rotation
    ///      - deadband_ry : Deadband for y-axis rotation
    ///      - deadband_rz : Deadband for z-axis rotation
    ///  - topic is the name of the topic on which joystick messages are
    ///      published.
    class SpaceNav
    {
      /// \brief Constructor.
      public: SpaceNav();

      /// \brief Destructor.
      public: virtual ~SpaceNav();

      /// \brief Open the space navigator interface, and start polling.
      /// \return True if the space navigator was successfully opened, or
      /// when the space navigator 3rd party library is not installed.
      /// False otherwise.
      public: bool Load();

      /// \brief Method executed in a separate thread to poll the space
      /// navigator for updates.
      private: void Run();

      /// \brief Apply deadband calculation to a raw joystick value.
      /// \param[in] _deadband Deadband value, between 0 and 1.
      /// \param[in] _value Joystick value, between 0 and 1.
      /// \return Result of deadband calculation.
      private: double Deadband(double _deadband, double _value) const;

      /// \brief Private data.
      private: SpaceNavPrivate *dataPtr;
    };
  }
}
#endif
