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
#ifndef GAZEBO_LINK_PLOT_3D_PLUGIN_HH_
#define GAZEBO_LINK_PLOT_3D_PLUGIN_HH_

#include <string>
#include <vector>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  /// \brief Information about each plot
  struct Plot3D
  {
    /// \brief Message
    ignition::msgs::Marker msg;

    physics::LinkPtr link;

    ignition::math::Pose3d pose;
  };

  /// \brief A plugin that simulates lift and drag.
  class GAZEBO_VISIBLE LinkPlot3DPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: LinkPlot3DPlugin();

    /// \brief Destructor.
    public: ~LinkPlot3DPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    protected: std::vector<Plot3D> plots;

    protected: ignition::transport::Node node;

    protected: physics::WorldPtr world;

    protected: int frequency;
  };
}
#endif
