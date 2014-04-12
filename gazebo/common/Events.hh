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
#ifndef _GAZEBO_COMMON_EVENTS_HH_
#define _GAZEBO_COMMON_EVENTS_HH_

#include <string>
#include <ignition/common.hh>

#include "gazebo/common/UpdateInfo.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_event
    /// \{

    /// \class Events Events.hh common/common.hh
    /// \brief An Event class to get notifications for simulator events
    class Events
    {
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the pause signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr ConnectPause(T _subscriber)
              { return pause.Connect(_subscriber); }

      /// \brief Disconnect a boost::slot the the pause signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectPause(
                  ignition::common::ConnectionPtr _subscriber)
              { pause.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the step signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr ConnectStep(T _subscriber)
              { return step.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the step signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectStep(
                  ignition::common::ConnectionPtr _subscriber)
              { step.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the stop signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr ConnectStop(T _subscriber)
              { return stop.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the stop signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectStop(
                  ignition::common::ConnectionPtr _subscriber)
              { stop.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the world created signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr
                ConnectWorldCreated(T _subscriber)
              { return worldCreated.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the world created signal
      public: static void DisconnectWorldCreated(
                  ignition::common::ConnectionPtr _subscriber)
              { worldCreated.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the add entity signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr
                ConnectCreateEntity(T _subscriber)
              { return entityCreated.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the add entity signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectCreateEntity(
                  ignition::common::ConnectionPtr _subscriber)
              { entityCreated.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the set selected entity
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr
                ConnectSetSelectedEntity(T _subscriber)
              { return setSelectedEntity.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the set selected entity
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectSetSelectedEntity(
                  ignition::common::ConnectionPtr _subscriber)
              { setSelectedEntity.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the delete entity
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr
                ConnectDeleteEntity(T _subscriber)
              { return deleteEntity.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the delete entity
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectDeleteEntity(
                  ignition::common::ConnectionPtr _subscriber)
              { deleteEntity.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the add entity signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr
                ConnectAddEntity(T _subscriber)
              { return addEntity.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the add entity signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectAddEntity(
                  ignition::common::ConnectionPtr _subscriber)
              { addEntity.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the world update start signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr
                ConnectWorldUpdateBegin(T _subscriber)
              { return worldUpdateBegin.Connect(_subscriber); }

      /// \brief Disconnect a boost::slot the the world update start signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectWorldUpdateBegin(
                  ignition::common::ConnectionPtr _subscriber);

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the the world update end signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr
                ConnectWorldUpdateEnd(T _subscriber)
              { return worldUpdateEnd.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the the world update end signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectWorldUpdateEnd(
                  ignition::common::ConnectionPtr _subscriber)
              { worldUpdateEnd.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Render start signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr
                ConnectPreRender(T _subscriber)
              { return preRender.Connect(_subscriber); }
      /// \brief Disconnect a render start signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectPreRender(
                  ignition::common::ConnectionPtr _subscriber)
              { preRender.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the render update signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr
                ConnectRender(T _subscriber)
              { return render.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the render update signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectRender(
                  ignition::common::ConnectionPtr _subscriber)
              { render.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the post render update signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr
                ConnectPostRender(T _subscriber)
              { return postRender.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the post render update signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectPostRender(
                  ignition::common::ConnectionPtr _subscriber)
              { postRender.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the diagnostic timer start signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr
                ConnectDiagTimerStart(T _subscriber)
              { return diagTimerStart.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the diagnostic timer start signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectDiagTimerStart(
                  ignition::common::ConnectionPtr _subscriber)
              { diagTimerStart.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot the diagnostic timer stop signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr
                ConnectDiagTimerStop(T _subscriber)
              { return diagTimerStop.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot the diagnostic timer stop signal
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectDiagTimerStop(
                  ignition::common::ConnectionPtr _subscriber)
              { diagTimerStop.Disconnect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a boost::slot to the sigint event
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ignition::common::ConnectionPtr
                ConnectSigInt(T _subscriber)
              { return sigInt.Connect(_subscriber); }
      /// \brief Disconnect a boost::slot to the sigint event
      /// \param[in] _subscriber the subscriber to this event
      public: static void DisconnectSigInt(
                  ignition::common::ConnectionPtr _subscriber)
              { sigInt.Disconnect(_subscriber); }

      /// \brief Pause signal
      public: static ignition::common::EventT<void (bool)> pause;

      /// \brief Step the simulation once signal
      public: static ignition::common::EventT<void ()> step;

      /// \brief Simulation stop signal
      public: static ignition::common::EventT<void ()> stop;

      /// \brief Simulation stop signal
      public: static ignition::common::EventT<void ()> sigInt;

      /// \brief A world has been created
      public: static ignition::common::EventT<void (std::string)> worldCreated;

      /// \brief An entity has been created
      public: static ignition::common::EventT<void (std::string)> entityCreated;

      /// \brief An entity has been selected
      public: static ignition::common::EventT<
              void(std::string, std::string)> setSelectedEntity;

      /// \brief An entity has been added
      public: static ignition::common::EventT<void (std::string)> addEntity;

      /// \brief An entity has been deleted
      public: static ignition::common::EventT<void (std::string)> deleteEntity;

      /// \brief World update has started
      public: static ignition::common::EventT
              <void (const gazebo::common::UpdateInfo &)> worldUpdateBegin;

      /// \brief World update has ended
      public: static ignition::common::EventT<void ()> worldUpdateEnd;

      /// \brief Pre-render
      public: static ignition::common::EventT<void ()> preRender;

      /// \brief Render
      public: static ignition::common::EventT<void ()> render;

      /// \brief Post-Render
      public: static ignition::common::EventT<void ()> postRender;

      /// \brief Diagnostic timer start
      public: static ignition::common::EventT<void (std::string)>
              diagTimerStart;

      /// \brief Diagnostic timer stop
      public: static ignition::common::EventT<void (std::string)>
              diagTimerStop;
    };
    /// \}
  }
}
#endif
