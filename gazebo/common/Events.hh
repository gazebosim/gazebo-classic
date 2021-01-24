/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef _EVENTS_HH_
#define _EVENTS_HH_

#include <string>
#include <sdf/sdf.hh>

#include "gazebo/common/Console.hh"
#include "gazebo/common/UpdateInfo.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace event
  {
    /// \addtogroup gazebo_event
    /// \{

    /// \class Events Events.hh common/common.hh
    /// \brief An Event class to get notifications for simulator events
    class GZ_COMMON_VISIBLE Events
    {
      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the pause signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectPause(T _subscriber)
              { return pause.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the step signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectStep(T _subscriber)
              { return step.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the stop signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectStop(T _subscriber)
              { return stop.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the world created signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectWorldCreated(T _subscriber)
              { return worldCreated.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the add entity signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectCreateEntity(T _subscriber)
              { return entityCreated.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the set selected entity signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectSetSelectedEntity(T _subscriber)
              { return setSelectedEntity.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the delete entity signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectDeleteEntity(T _subscriber)
              { return deleteEntity.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the add entity signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectAddEntity(T _subscriber)
              { return addEntity.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the world update start signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectWorldUpdateBegin(T _subscriber)
              { return worldUpdateBegin.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the before physics update signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      ///
      /// The signal is called after collision detection has finished and before
      /// the physics update step. So you can e.g. change some forces depending
      /// on the collisions that have occured.
      public: template<typename T>
              static ConnectionPtr ConnectBeforePhysicsUpdate(T _subscriber)
              { return beforePhysicsUpdate.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the world update end signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectWorldUpdateEnd(T _subscriber)
              { return worldUpdateEnd.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect to the world reset signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectWorldReset(T _subscriber)
              { return worldReset.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect to the time reset signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectTimeReset(T _subscriber)
              { return timeReset.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect to the remove sensor signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectRemoveSensor(T _subscriber)
              { return removeSensor.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect to the create sensor signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectCreateSensor(T _subscriber)
              { return createSensor.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief End of prerender phase signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectPreRenderEnded(T _subscriber)
              { return preRenderEnded.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Render start signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectPreRender(T _subscriber)
              { return preRender.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the render update signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectRender(T _subscriber)
              { return render.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the post render update signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectPostRender(T _subscriber)
              { return postRender.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the diagnostic timer start signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectDiagTimerStart(T _subscriber)
              { return diagTimerStart.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the diagnostic timer stop signal
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectDiagTimerStop(T _subscriber)
              { return diagTimerStop.Connect(_subscriber); }

      //////////////////////////////////////////////////////////////////////////
      /// \brief Connect a callback to the sigint event
      /// \param[in] _subscriber the subscriber to this event
      /// \return a connection
      public: template<typename T>
              static ConnectionPtr ConnectSigInt(T _subscriber)
              { return sigInt.Connect(_subscriber); }

      /// \brief Pause signal
      public: static EventT<void (bool)> pause;

      /// \brief Step the simulation once signal
      public: static EventT<void ()> step;

      /// \brief Simulation stop signal
      public: static EventT<void ()> stop;

      /// \brief Simulation SIGINT/SIGTERM signal
      public: static EventT<void ()> sigInt;

      /// \brief A world has been created
      public: static EventT<void (std::string)> worldCreated;

      /// \brief An entity has been created
      public: static EventT<void (std::string)> entityCreated;

      /// \brief An entity has been selected
      public: static EventT<void (std::string, std::string)> setSelectedEntity;

      /// \brief An entity has been added
      public: static EventT<void (std::string)> addEntity;

      /// \brief An entity has been deleted
      public: static EventT<void (std::string)> deleteEntity;

      /// \brief World update has started
      public: static EventT<void (const common::UpdateInfo &)> worldUpdateBegin;

      /// \brief Collision detection has been done, physics update not yet
      public: static EventT<void (const common::UpdateInfo &)>
                beforePhysicsUpdate;

      /// \brief World update has ended
      public: static EventT<void ()> worldUpdateEnd;

      /// \brief World reset signal
      public: static EventT<void ()> worldReset;

      /// \brief Time reset signal
      public: static EventT<void ()> timeReset;

      /// \brief Pre-render
      public: static EventT<void ()> preRender;

      /// \brief End of Pre-render
      public: static EventT<void ()> preRenderEnded;

      /// \brief Render
      public: static EventT<void ()> render;

      /// \brief Post-Render
      public: static EventT<void ()> postRender;

      /// \brief Diagnostic timer start
      public: static EventT<void (std::string)> diagTimerStart;

      /// \brief Diagnostic timer stop
      public: static EventT<void (std::string)> diagTimerStop;

      /// \brief Remove a sensor
      ///
      ///  * Parameter 1 (string): Name of the sensor to remove.
      public: static EventT<void (std::string)> removeSensor;

      /// \brief Create a sensor
      /// * Parameter 1 (sdf::ElementPtr): The SDF element that describes
      ///   the sensor.
      /// * Parameter 2 (std::string): Name of the world in which to create
      ///   the sensor.
      /// * Parameter 3 (std::string): The scoped parent name
      ///   (model::link).
      /// * Parameter 4 (uint32_t): ID of the sensor
      public: static EventT<void (sdf::ElementPtr,
                  const std::string &,
                  const std::string &,
                  const uint32_t)> createSensor;
    };
    /// \}
  }
}
#endif
