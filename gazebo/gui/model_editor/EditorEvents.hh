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
#ifndef _EDITOR_EVENTS_HH_
#define _EDITOR_EVENTS_HH_

#include <string>
#include "common/Event.hh"
/*#include "msgs/msgs.hh"
#include "math/Vector3.hh"
#include "math/Pose.hh"*/

namespace gazebo
{
  namespace gui
  {
    class Events
    {
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the create editor item signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static event::ConnectionPtr ConnectCreateEditorItem(T _subscriber)
              { return createEditorItem.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot to the create editor item signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectCreateEditorItem(
                  event::ConnectionPtr _subscriber)
              { createEditorItem.Disconnect(_subscriber); }

      public: template<typename T>
              static event::ConnectionPtr ConnectFinishModel(T _subscriber)
              { return finishModel.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot to the finish model signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectFinishModel(
                  event::ConnectionPtr _subscriber)
              { finishModel.Disconnect(_subscriber); }

      public: template<typename T>
              static event::ConnectionPtr ConnectDiscardModel(T _subscriber)
              { return discardModel.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot to the discard model signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectDiscardModel(
                  event::ConnectionPtr _subscriber)
              { discardModel.Disconnect(_subscriber); }

      public: template<typename T>
              static event::ConnectionPtr ConnectChangeLevel(T _subscriber)
              { return changeLevel.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot to the change level signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectChangeLevel(
                  event::ConnectionPtr _subscriber)
              { changeLevel.Disconnect(_subscriber); }

      public: template<typename T>
              static event::ConnectionPtr ConnectAddLevel(T _subscriber)
              { return addLevel.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot to the add level signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectAddLevel(
                  event::ConnectionPtr _subscriber)
              { addLevel.Disconnect(_subscriber); }

      public: template<typename T>
              static event::ConnectionPtr ConnectChangeLevelName(T _subscriber)
              { return changeLevelName.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot to the change level name signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectChangeLevelName(
                  event::ConnectionPtr _subscriber)
              { changeLevelName.Disconnect(_subscriber); }

      /// \brief An editor item is to be created
      public: static event::EventT<void (std::string)> createEditorItem;

      /// \brief A model is to be created
      public: static event::EventT<void (std::string, std::string)>
          finishModel;

      /// \brief A model is to be created
      public: static event::EventT<void ()> discardModel;

      /// \brief The current level has been changed
      public: static event::EventT<void (int)> changeLevel;

      /// \brief A new level is to be added
      public: static event::EventT<void (int, std::string)> addLevel;

      /// \brief The current level name has been changed
      public: static event::EventT<void (int, std::string)> changeLevelName;

/*      public: template<typename T>
              static event::ConnectionPtr ConnectCreateBuildingPart(T _subscriber)
              { return createBuildingPart.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot to the create editor item signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectCreateBuildingPart(
                  event::ConnectionPtr _subscriber)
              { createBuildingPart.Disconnect(_subscriber); }

      public: template<typename T>
              static event::ConnectionPtr ConnectSetBuildingPartPose(T _subscriber)
              { return setBuildingPartPose.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot to the set building part pose signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectSetBuildingPartPose(
                  event::ConnectionPtr _subscriber)
              { setBuildingPartPose.Disconnect(_subscriber); }

      public: template<typename T>
              static event::ConnectionPtr ConnectSetBuildingPartSize(T _subscriber)
              { return setBuildingPartSize.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot to the set building part pose signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectSetBuildingPartSize(
                  event::ConnectionPtr _subscriber)
              { setBuildingPartSize.Disconnect(_subscriber); }


      /// \brief A building part has been created
      public: static event::EventT<void (std::string)> createBuildingPart;

      /// \brief A building part pose has been modified
      public: static event::EventT<void (std::string, math::Pose)>
          setBuildingPartPose;

      /// \brief A building part size has been modified
      public: static event::EventT<void (std::string, math::Vector3)>
          setBuildingPartSize;*/
    };
  }
}
#endif
