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

#include "gazebo/physics/EntityPrivate.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Protected Link data
    class LinkProtected : public EntityProtected
    {
      /// \brief Inertial properties.
      protected: InertialPtr inertial;

      /// \brief Center of gravity visual elements.
      protected: std::vector<std::string> cgVisuals;

      /// \def Visuals_M
      /// \brief Map of unique ID to visual message.
      typedef std::map<uint32_t, msgs::Visual> Visuals_M;

      /// \brief Link visual elements.
      protected: Visuals_M visuals;

      /// \brief Linear acceleration.
      protected: math::Vector3 linearAccel;

      /// \brief Angular acceleration.
      protected: math::Vector3 angularAccel;

      /// \brief Offsets for the attached models.
      protected: std::vector<math::Pose> attachedModelsOffset;

      /// \brief This flag is set to true when the link is initialized.
      protected: bool initialized;
    };

    /// \brief Private Link data
    class LinkPrivate
    {
      /// \brief Event used when the link is enabled or disabled.
      private: event::EventT<void (bool)> enabledSignal;

      /// \brief This flag is used to trigger the enabled
      private: bool enabled;

      /// \brief Names of all the sensors attached to the link.
      private: std::vector<std::string> sensors;

      /// \brief All the parent joints.
      private: std::vector<JointPtr> parentJoints;

      /// \brief All the child joints.
      private: std::vector<JointPtr> childJoints;

      /// \brief All the attached models.
      private: std::vector<ModelPtr> attachedModels;

      /// \brief Link data publisher
      private: transport::PublisherPtr dataPub;

      /// \brief Link data message
      private: msgs::LinkData linkDataMsg;

      /// \brief True to publish data, false otherwise
      private: bool publishData;

      /// \brief Mutex to protect the publishData variable
      private: boost::recursive_mutex *publishDataMutex;

      /// \brief Cached list of collisions. This is here for performance.
      private: Collision_V collisions;

      /// \brief Wrench subscriber.
      private: transport::SubscriberPtr wrenchSub;

      /// \brief Vector of wrench messages to be processed.
      private: std::vector<msgs::Wrench> wrenchMsgs;

      /// \brief Mutex to protect the wrenchMsgs variable.
      private: boost::mutex wrenchMsgMutex;

      /// \brief All the attached batteries.
      private: std::vector<common::BatteryPtr> batteries;

#ifdef HAVE_OPENAL
      /// \brief All the audio sources
      private: std::vector<util::OpenALSourcePtr> audioSources;

      /// \brief An audio sink
      private: util::OpenALSinkPtr audioSink;

      /// \brief Subscriber to contacts with this collision. Used for audio
      /// playback.
      private: transport::SubscriberPtr audioContactsSub;
#endif
    };
  }
}
#endif
