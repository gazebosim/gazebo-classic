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

#ifndef _GAZEBO_CESSNA_PLUGIN_HH_
#define _GAZEBO_CESSNA_PLUGIN_HH_

#include <mutex>
#include <string>
#include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>

namespace gazebo
{
  /// \brief A simple object follower that finds closest object from an image
  /// and commands a differential drive vehicle to move towards the object.
  class GAZEBO_VISIBLE FollowerPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: FollowerPlugin();

    /// \brief Destructor.
    public: virtual ~FollowerPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Update the control surfaces controllers.
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Callback executed when a new image message is received.
    /// \param[in] _msg New image message.
    private: void OnImage(ConstImageStampedPtr &_msg);

    /// \brief Find a depth sensor in this model.
    private: void FindSensor();

    /// \brief Find revolute joints in the model.
    private: void FindJoints();

    /// \brief Update the follower.
    private: void UpdateFollower();

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Node used for using Gazebo communications.
    private: transport::NodePtr node;

    /// \brief Subscriber pointer.
    private: transport::SubscriberPtr imageSub;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Update mutex.
    private: std::mutex mutex;

    /// \brief Input depth image topic.
    private: std::string imageTopic;

    /// \brief Local copy of input image.
    private: msgs::Image imageMsg;

    /// \brief Revolute joints for moving the wheels of the vehicle.
    private: physics::JointPtr leftJoint, rightJoint;

    /// \brief Left/Right wheel speed.
    private: double wheelSpeed[2];

    /// \brief Wheel separation.
    private: double wheelSeparation;

    /// \brief Wheel radius.
    private: double wheelRadius;
  };
}
#endif
