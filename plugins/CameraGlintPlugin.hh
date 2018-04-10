/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS_CAMERAGLINTPLUGIN_HH_
#define GAZEBO_PLUGINS_CAMERAGLINTPLUGIN_HH_

#include "gazebo/common/Plugin.hh"

namespace gazebo
{
  // Forward declaration
  class CameraGlintPluginPrivate;

  /// \brief Attach a CameraGlintPlugin to each camera which needs to simulate
  /// glints from the sun. If your world has any CameraGlintPlugin objects, it
  /// also needs one CameraGlintDetectionPlugin attached to the world. One
  /// CameraGlintDetectionPlugin in the world is sufficient for any number of
  /// CameraGlintPlugins.
  class GAZEBO_VISIBLE CameraGlintPlugin : public SensorPlugin
  {
    /// \brief Default constructor
    public: CameraGlintPlugin();

    /// \brief Virtual destructor
    public: virtual ~CameraGlintPlugin();

    /// \brief Tie into the camera sensor in order to render glints
    public: virtual void Init() override;

    /// \brief Detect the camera sensor that we are connected to
    public: virtual void Load(
      sensors::SensorPtr _sensor,
      sdf::ElementPtr _sdf) override;

    /// \brief PIMPL pointer to implementation
    private: std::unique_ptr<CameraGlintPluginPrivate> dataPtr;
  };
}

#endif
