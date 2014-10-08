/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
    class GAZEBO_VISIBLE Events
    {
      /////////////////////////////////////////////////
      /// \brief Connect a boost::slot the add entity signal
      public: template<typename T>
              static event::ConnectionPtr ConnectCreateEntity(T _subscriber)
              { return createEntity.Connect(_subscriber); }
      public: static void DisconnectCreateEntity(
                  event::ConnectionPtr _subscriber)
              { createEntity.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot to the move mode signal
      public: template<typename T>
              static event::ConnectionPtr ConnectMoveMode(T _subscriber)
              { return moveMode.Connect(_subscriber); }
      public: static void DisconnectMoveMode(event::ConnectionPtr _subscriber)
              { moveMode.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot to the manip mode signal
      public: template<typename T>
              static event::ConnectionPtr ConnectManipMode(T _subscriber)
              {return manipMode.Connect(_subscriber);}
      public: static void DisconnectManipMode(event::ConnectionPtr _subscriber)
              {manipMode.Disconnect(_subscriber);}

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot to the fullscreen signal
      public: template<typename T>
              static event::ConnectionPtr ConnectFullScreen(T _subscriber)
              { return fullScreen.Connect(_subscriber); }
      public: static void DisconnectFullScreen(event::ConnectionPtr _subscriber)
              { fullScreen.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot to the view FPS signal
      public: template<typename T>
              static event::ConnectionPtr ConnectFPS(T _subscriber)
              { return fps.Connect(_subscriber); }
      public: static void DisconnectFPS(event::ConnectionPtr _subscriber)
              { fps.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot to the view Orbit signal
      public: template<typename T>
              static event::ConnectionPtr ConnectOrbit(T _subscriber)
              { return orbit.Connect(_subscriber); }
      public: static void DisconnectOrbit(event::ConnectionPtr _subscriber)
              { orbit.Disconnect(_subscriber); }
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot to the view KeyPress signal
      public: template<typename T>
              static event::ConnectionPtr ConnectKeyPress(T _subscriber)
              { return keyPress.Connect(_subscriber); }
      public: static void DisconnectKeyPress(event::ConnectionPtr _subscriber)
              { keyPress.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot to the light update signal
      public: template<typename T>
              static event::ConnectionPtr ConnectLightUpdate(T _subscriber)
              { return lightUpdate.Connect(_subscriber); }
      public: static void DisconnectLightUpdate(
                  event::ConnectionPtr _subscriber)
              { lightUpdate.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      public: template<typename T>
              static event::ConnectionPtr ConnectModelUpdate(T _subscriber)
              { return modelUpdate.Connect(_subscriber); }
      public: static void DisconnectModelUpdate(
                  event::ConnectionPtr _subscriber)
              { modelUpdate.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot to the input step size signal
      public: template<typename T>
              static event::ConnectionPtr ConnectInputStepSize(T _subscriber)
              { return inputStepSize.Connect(_subscriber); }
      public: static void DisconnectInputStepSize(
              event::ConnectionPtr _subscriber)
              { inputStepSize.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot to the follow signal
      public: template<typename T>
              static event::ConnectionPtr ConnectFollow(T _subscriber)
              { return follow.Connect(_subscriber); }
      public: static void DisconnectFollow(
              event::ConnectionPtr _subscriber)
              { follow.Disconnect(_subscriber); }

      ///  that indicates the user is moving the camera
      public: static event::EventT<void (bool)>  moveMode;

      ///  that indicates the user is manipulating an object
      public: static event::EventT<void (std::string)>  manipMode;

      public: static event::EventT<void (std::string,
                                         std::string)> createEntity;

      public: static event::EventT<void (const msgs::Model &)> modelUpdate;

      /// \brief A event to notify light updates.
      public: static event::EventT<void (const msgs::Light &)> lightUpdate;

      public: static event::EventT<void (bool)> fullScreen;
      public: static event::EventT<void ()> fps;
      public: static event::EventT<void ()> orbit;

      /// \brief Event triggered when the user follows a model. The model
      /// name is given as the function parameter.
      public: static event::EventT<void (const std::string &)> follow;

      public: static event::EventT<void (std::string)> keyPress;

      /// \brief Step size changed event
      public: static event::EventT<void (int)> inputStepSize;
    };
  }
}
#endif
