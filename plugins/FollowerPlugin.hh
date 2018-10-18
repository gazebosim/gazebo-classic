/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_FOLLOWER_PLUGIN_HH_
#define _GAZEBO_FOLLOWER_PLUGIN_HH_

#include <memory>
#include <string>

#include <sdf/sdf.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/PhysicsTypes.hh>

namespace gazebo
{
  // Forward declare private class
  struct FollowerPluginPrivate;

  /// \brief A simple object follower that finds the closest object in a
  /// depth image and commands a differential drive vehicle to move towards the
  /// object. The plugin is essentially an integration of the DiffDrivePlugin
  /// and DepthCameraPlugin.
  class GAZEBO_VISIBLE FollowerPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: FollowerPlugin();

    /// \brief Destructor.
    public: ~FollowerPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    // Documentation Unherited.
    public: virtual void Reset();

    /// \brief Callback when a new depth camera frame is received.
    /// \param[in] _image Depth image data buffer
    /// \param[in] _width Image width
    /// \param[in] _height Image height
    /// \param[in] _depth Image depth
    /// \param[in] _format Image format
    private: void OnNewDepthFrame(const float *_image,
                const unsigned int _width, const unsigned int _height,
                const unsigned int _depth, const std::string &_format);

    /// \brief Update the follower.
    private: void OnUpdate();

    /// \brief Find a depth sensor in the model.
    /// \param[in] _model Model to search for depth sensor.
    /// \return True if depth sensor is found.
    private: bool FindSensor(const physics::ModelPtr &_model);

    /// \brief Find revolute joints in the model.
    private: void FindJoints();

    /// \brief Update the follower.
    private: void UpdateFollower();

    /// \brief Pointer to private data
    private: std::unique_ptr<FollowerPluginPrivate> dataPtr;
  };
}
#endif
