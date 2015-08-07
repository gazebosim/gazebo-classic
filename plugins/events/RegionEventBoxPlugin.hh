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

#ifndef _GAZEBO_REGION_EVENT_BOX_PLUGIN_HH_
#define _GAZEBO_REGION_EVENT_BOX_PLUGIN_HH_

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include <map>
#include <string>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

#include "EventSource.hh"

namespace gazebo
{
  class RegionEventBoxPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: RegionEventBoxPlugin();

    // \brief Documentation Inherited.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Callback when a model message is received.
    /// \param[in] _msg model msg
    public: void OnModelMsg(ConstModelPtr &_msg);

    /// \brief Updates the box event plugin at every physics iteration
    /// \param[in] _info Update info
    public: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Check if point is contained within a 3D box region.
    /// \param[in] _point 3D Point in world space.
    /// \param[in] _box World axis-aligned box.
    /// \param[in] _pose Pose of the model representing the box region.
    private: bool PointInRegion(const ignition::math::Vector3d &_point,
        const ignition::math::Box &_box, const ignition::math::Pose3d &_pose);

    /// \brief Update box region dimensions and pose.
    /// \param[in] _scale New scale
    /// \param[in] _pose New pose
    private: bool UpdateRegion(const ignition::math::Vector3d &_scale,
        const ignition::math::Pose3d &_pose);

    /// \brief Send event when model enters box region
    /// \param[in] _model Model that entered the box region.
    private: void SendEnteringRegionEvent(physics::ModelPtr _model);

    /// \brief Send event when model exits region
    /// \param[in] _model Model that exit the box region.
    private: void SendExitingRegionEvent(physics::ModelPtr _model);

    /// \brief Pointer to the World.
    private: physics::WorldPtr world;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief WorldUpdateBegin connection Event.
    private: event::ConnectionPtr updateConnection;

    /// \brief Name of model.
    private: std::string modelName;

    /// \brief Transport node pointer.
    public: transport::NodePtr node;

    /// \brief Mutex to protect incoming messages
    public: boost::mutex receiveMutex;

    /// \brief Box region initial size
    private: ignition::math::Vector3d boxSize;

    /// \brief Box region scale
    private: ignition::math::Vector3d boxScale;

    /// \brief Box region pose.
    private: ignition::math::Pose3d boxPose;

    /// \brief Box region
    private: ignition::math::Box box;

    /// \brief Subscriber to model/info topic.
    private: transport::SubscriberPtr modelSub;

    /// \brief Flag set when box region size or pose has changed.
    private: bool hasStaleSizeAndPose;

    /// \brief A map that stores model names and the sim time when they entered
    /// the box region.
    private: std::map<std::string, common::Time> insiders;

    /// \brief Publisher to the sim event topic
    private: transport::PublisherPtr eventPub;

    /// \brief Pointer to event source object that emits sim events
    private: gazebo::EventSourcePtr eventSource;
  };
}

#endif
