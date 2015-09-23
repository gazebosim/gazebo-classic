/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _GUI_EVENTS_HH_
#define _GUI_EVENTS_HH_

#include <string>
#include "gazebo/common/Event.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class GZ_GUI_VISIBLE Events
    {
      /////////////////////////////////////////////////
      /// \brief Connect a signal the add entity signal
      public: template<typename T>
              static event::ConnectionPtr ConnectCreateEntity(T _subscriber)
              { return createEntity.Connect(_subscriber); }

      /// \brief Disconnect a signal from add entity signal
      public: static void DisconnectCreateEntity(
                  event::ConnectionPtr _subscriber)
              { createEntity.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the move mode signal
      public: template<typename T>
              static event::ConnectionPtr ConnectMoveMode(T _subscriber)
              { return moveMode.Connect(_subscriber); }

      /// \brief Disconnect a signal from the move mode signal
      public: static void DisconnectMoveMode(event::ConnectionPtr _subscriber)
              { moveMode.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the manip mode signal
      public: template<typename T>
              static event::ConnectionPtr ConnectManipMode(T _subscriber)
              {return manipMode.Connect(_subscriber);}

      /// \brief Disconnect a signal from the manip mode signal
      public: static void DisconnectManipMode(event::ConnectionPtr _subscriber)
              {manipMode.Disconnect(_subscriber);}

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the align mode signal
      public: template<typename T>
              static event::ConnectionPtr ConnectAlignMode(T _subscriber)
              {return alignMode.Connect(_subscriber);}

      /// \brief Disconnect a signal from the align mode signal
      public: static void DisconnectAlignMode(event::ConnectionPtr _subscriber)
              {alignMode.Disconnect(_subscriber);}

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the fullscreen signal
      public: template<typename T>
              static event::ConnectionPtr ConnectFullScreen(T _subscriber)
              { return fullScreen.Connect(_subscriber); }

      /// \brief Disconnect a signal from the fullscreen signal
      public: static void DisconnectFullScreen(event::ConnectionPtr _subscriber)
              { fullScreen.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the show toolbars signal
      public: template<typename T>
              static event::ConnectionPtr ConnectShowToolbars(T _subscriber)
              { return showToolbars.Connect(_subscriber); }

      /// \brief Disconnect a signal from the show toolbars signal
      public: static void DisconnectShowToolbars(event::ConnectionPtr
          _subscriber)
              { showToolbars.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the view FPS signal
      public: template<typename T>
              static event::ConnectionPtr ConnectFPS(T _subscriber)
              { return fps.Connect(_subscriber); }

      /// \brief Disconnect a signal from the view FPS signal
      public: static void DisconnectFPS(event::ConnectionPtr _subscriber)
              { fps.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the view Orbit signal
      public: template<typename T>
              static event::ConnectionPtr ConnectOrbit(T _subscriber)
              { return orbit.Connect(_subscriber); }

      /// \brief Disconnect a signal from the view Orbit signal
      public: static void DisconnectOrbit(event::ConnectionPtr _subscriber)
              { orbit.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the view KeyPress signal
      public: template<typename T>
              static event::ConnectionPtr ConnectKeyPress(T _subscriber)
              { return keyPress.Connect(_subscriber); }

      /// \brief Disconnect a signal from the view KeyPress signal
      public: static void DisconnectKeyPress(event::ConnectionPtr _subscriber)
              { keyPress.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the light update signal
      public: template<typename T>
              static event::ConnectionPtr ConnectLightUpdate(T _subscriber)
              { return lightUpdate.Connect(_subscriber); }

      /// \brief Disconnect a signal from the light update signal
      public: static void DisconnectLightUpdate(
                  event::ConnectionPtr _subscriber)
              { lightUpdate.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the model update signal
      public: template<typename T>
              static event::ConnectionPtr ConnectModelUpdate(T _subscriber)
              { return modelUpdate.Connect(_subscriber); }

      /// \brief Disconnect a signal from the model update signal
      public: static void DisconnectModelUpdate(
                  event::ConnectionPtr _subscriber)
              { modelUpdate.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the input step size signal
      public: template<typename T>
              static event::ConnectionPtr ConnectInputStepSize(T _subscriber)
              { return inputStepSize.Connect(_subscriber); }

      /// \brief Disconnect a signal from the input step size signal
      public: static void DisconnectInputStepSize(
              event::ConnectionPtr _subscriber)
              { inputStepSize.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the follow signal
      public: template<typename T>
              static event::ConnectionPtr ConnectFollow(T _subscriber)
              { return follow.Connect(_subscriber); }

      /// \brief Disconnect a signal from the follow signal
      public: static void DisconnectFollow(
              event::ConnectionPtr _subscriber)
              { follow.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the edit model signal
      public: template<typename T>
              static event::ConnectionPtr ConnectEditModel(T _subscriber)
              { return editModel.Connect(_subscriber); }

      /// \brief Disconnect a signal from the edit model signal
      public: static void DisconnectEditModel(
              event::ConnectionPtr _subscriber)
              { editModel.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the window mode signal
      public: template<typename T>
              static event::ConnectionPtr ConnectWindowMode(T _subscriber)
              { return windowMode.Connect(_subscriber); }

      /// \brief Disconnect a signal from the window mode signal
      public: static void DisconnectWindowMode(
              event::ConnectionPtr _subscriber)
              { windowMode.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the main window ready signal
      public: template<typename T>
              static event::ConnectionPtr ConnectMainWindowReady(T _subscriber)
              { return mainWindowReady.Connect(_subscriber); }

      /// \brief Disconnect a signal from the main window ready signal
      public: static void DisconnectMainWindowReady(
              event::ConnectionPtr _subscriber)
              { mainWindowReady.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to toggle the GUI's left hand pane signal
      public: template<typename T>
              static event::ConnectionPtr ConnectLeftPaneVisibility
                (T _subscriber)
                { return leftPaneVisibility.Connect(_subscriber); }

      /// \brief Disconnect a signal to toggle the GUI's left hand pane signal
      public: static void DisconnectLeftPaneVisibility(
              event::ConnectionPtr _subscriber)
              { leftPaneVisibility.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a signal to the scale entity signal
      public: template<typename T>
              static event::ConnectionPtr ConnectScaleEntity(T _subscriber)
              { return scaleEntity.Connect(_subscriber); }

      /// \brief Disconnect a signal from the scale entity signal
      public: static void DisconnectScaleEntity(
              event::ConnectionPtr _subscriber)
              { scaleEntity.Disconnect(_subscriber); }

      /// \brief Indicates the user is moving the camera
      public: static event::EventT<void (bool)>  moveMode;

      /// \brief indicates the user is manipulating an object
      public: static event::EventT<void (std::string)>  manipMode;

      /// \brief indicates the user is aligning objects
      public: static event::EventT<void (std::string,
                  std::string, std::string, bool)> alignMode;

      /// \brief indicates an entity has been created
      public: static event::EventT<void (std::string,
                                         std::string)> createEntity;

      /// \brief indicates a model has been updated
      public: static event::EventT<void (const msgs::Model &)> modelUpdate;

      /// \brief An event to notify light updates.
      public: static event::EventT<void (const msgs::Light &)> lightUpdate;

      /// \brief An event to trigger full screen mode.
      public: static event::EventT<void (bool)> fullScreen;

      /// \brief An event to trigger show toolbars.
      public: static event::EventT<void (bool)> showToolbars;

      /// \brief An event to enable first-person-shooter view control.
      public: static event::EventT<void ()> fps;

      /// \brief An event to enable orbit view control.
      public: static event::EventT<void ()> orbit;

      /// \brief Event triggered when the user follows a model. The model
      /// name is given as the function parameter.
      public: static event::EventT<void (const std::string &)> follow;

      /// \brief Event triggered when the user selects edit a model. The model
      /// name is given as the function parameter.
      public: static event::EventT<void (const std::string &)> editModel;

      /// \brief Event triggered when changing window mode. Possible modes are:
      /// "Simulation", "ModelEditor", "LogPlayback"
      public: static event::EventT<void (const std::string &)> windowMode;

      /// \brief Event triggered when a key is pressed
      public: static event::EventT<void (std::string)> keyPress;

      /// \brief Step size changed event
      public: static event::EventT<void (int)> inputStepSize;

      /// \brief Used to set whether the GUI's left pane is visible
      public: static event::EventT<void (bool)> leftPaneVisibility;

      /// \brief Main window ready event.
      public: static event::EventT<void ()> mainWindowReady;

      /// \brief Scale entity event.
      public: static event::EventT<void (const std::string &,
          const math::Vector3 &)> scaleEntity;
    };
  }
}
#endif
