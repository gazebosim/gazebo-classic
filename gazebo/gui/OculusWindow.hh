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

#ifndef GAZEBO_GUI_OCULUS_WINDOW_HH_
#define GAZEBO_GUI_OCULUS_WINDOW_HH_

#include <string>
#include <thread>

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

    /// \class OculusWindow OculusWindow.hh gui/OculusWindow.hh
    /// \brief A widget that renders a camera view suitable for the Oculus
    /// Rift.
    class GZ_GUI_VISIBLE OculusWindow : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: OculusWindow(int _x, int _y, const std::string &_visual,
                  QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~OculusWindow();

      /// \brief Initialize Oculus and create a camera.
      /// \return True when Oculus is connected and the camera was created.
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

      /// \brief Used to attach the Oculus to the visual link.
      private: void AttachCameraToVisual();

      /// \brief QT widget used for rendering images.
      private: QFrame *renderFrame;

      /// \brief Identifier of the window for this oculus camera.
      private: int windowId;

      /// \brief Oculus camera pointer,
      private: rendering::OculusCameraPtr oculusCamera;

      /// \brief Scene pointer.
      private: rendering::ScenePtr scene;

      /// \brief Use full screen when true.
      private: bool isFullScreen;

      /// \brief X top left coordinate of the oculus window.
      private: int xPos;

      /// \brief Y top left coordinate of the oculus window.
      private: int yPos;

      /// \brief Oculus camera will be attached to this visual link.
      private: std::string visualName;

      /// \brief Thread to attach oculus camera to visual
      private: std::thread *attachCameraThread;
    };
  }
}
#endif
