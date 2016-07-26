/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_OSVR_WINDOW_HH_
#define _GAZEBO_OSVR_WINDOW_HH_

#include <thread>
#include <string>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class OSVRWindow OSVRWindow.hh gui/OSVRWindow.hh
    /// \brief A widget that renders a camera view suitable for an OSVR
    /// headset.
    class GZ_GUI_VISIBLE OSVRWindow : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: OSVRWindow(int _x, int _y, const std::string &_visual,
                  QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~OSVRWindow();

      /// \brief Initialize headset and create a camera.
      /// \return True when OSVR is connected and the camera was created.
      public: bool CreateCamera();

      // Documentation inherited.
      protected: virtual void showEvent(QShowEvent *_e);

      // Documentation inherited.
      protected: virtual void resizeEvent(QResizeEvent *_e);

      // Documentation inherited.
      protected: void keyPressEvent(QKeyEvent *_event);

      /// \brief Get the Ogre handler.
      /// \return std::string representation of the Ogre handler.
      private: std::string GetOgreHandle() const;

      /// \brief Used to attach the headset to the visual link.
      private: void AttachCameraToVisual();

      /// \brief QT widget used for rendering images.
      private: QFrame *renderFrame;

      /// \brief Identifier of the window for this headset camera.
      private: int windowId;

      /// \brief OSVR camera pointer,
      private: rendering::OSVRCameraPtr osvrCamera;

      /// \brief Scene pointer.
      private: rendering::ScenePtr scene;

      /// \brief Use full screen when true.
      private: bool isFullScreen;

      /// \brief X top left coordinate of the OSVR window.
      private: int xPos;

      /// \brief Y top left coordinate of the OSVR window.
      private: int yPos;

      /// \brief OSVR camera will be attached to this visual link.
      private: std::string visualName;

      /// \brief Thread to attach OSVR camera to visual
      private: std::thread *attachCameraThread;
    };
  }
}
#endif
