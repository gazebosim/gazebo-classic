/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_PHYSICS_LINK_PRIVATE_HH_
#define _GAZEBO_PHYSICS_LINK_PRIVATE_HH_
#include <vector>
#include <map>
#include <string>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/transport/transport.hh"
#include "gazebo/util/OpenAL.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/Inertial.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/EntityPrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief Private Link data
    class LinkPrivate : public EntityPrivate
    {
      /// \brief Inertial properties.
      public: Inertial inertial;

      /// \brief Center of gravity visual elements.
      public: std::vector<std::string> cgVisuals;

      /// \def Visuals_M
      /// \brief Map of unique ID to visual message.
      typedef std::map<uint32_t, msgs::Visual> Visuals_M;

      /// \brief Link visual elements.
      public: Visuals_M visuals;

      /// \brief Linear acceleration.
      public: ignition::math::Vector3d linearAccel;

      /// \brief Angular acceleration.
      public: ignition::math::Vector3d angularAccel;

      /// \brief Offsets for the attached models.
      public: std::vector<ignition::math::Pose3d> attachedModelsOffset;

      /// \brief This flag is set to true when the link is initialized.
      public: bool initialized = false;

      /// \brief Event used when the link is enabled or disabled.
      public: event::EventT<void (bool)> enabledSignal;

      /// \brief This flag is used to trigger the enabled
      public: bool enabled;

      /// \brief Names of all the sensors attached to the link.
      public: std::vector<std::string> sensors;

      /// \brief All the parent joints.
      public: std::vector<JointPtr> parentJoints;

      /// \brief All the child joints.
      public: std::vector<JointPtr> childJoints;

      /// \brief All the attached models.
      public: std::vector<ModelPtr> attachedModels;

      /// \brief Link data publisher
      public: transport::PublisherPtr dataPub;

      /// \brief Link data message
      public: msgs::LinkData linkDataMsg;

      /// \brief True to publish data, false otherwise
      public: bool publishData;

      /// \brief Mutex to protect the publishData variable
      public: std::recursive_mutex publishDataMutex;

      /// \brief Cached list of collisions. This is here for performance.
      public: Collision_V collisions;

      /// \brief Wrench subscriber.
      public: transport::SubscriberPtr wrenchSub;

      /// \brief Vector of wrench messages to be processed.
      public: std::vector<msgs::Wrench> wrenchMsgs;

      /// \brief Mutex to protect the wrenchMsgs variable.
      public: std::mutex wrenchMsgMutex;

      /// \brief All the attached batteries.
      public: std::vector<common::BatteryPtr> batteries;

#ifdef HAVE_OPENAL
      /// \brief All the audio sources
      public: std::vector<util::OpenALSourcePtr> audioSources;

      /// \brief An audio sink
      public: util::OpenALSinkPtr audioSink;

      /// \brief Subscriber to contacts with this collision. Used for audio
      /// playback.
      public: transport::SubscriberPtr audioContactsSub;
#endif
    };
  }
}
#endif
