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
#ifndef GAZEBO_PLUGINS_GRAVITYCOMPENSATIONPLUGIN_HH_
#define GAZEBO_PLUGINS_GRAVITYCOMPENSATIONPLUGIN_HH_

#include <memory>

#include <sdf/sdf.hh>

#include <dart/common/LocalResourceRetriever.hpp>

#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  // Forward declare private data class.
  class GravityCompensationPluginPrivate;

  class GAZEBO_VISIBLE ModelResourceRetriever :
      public virtual dart::common::LocalResourceRetriever
  {
    /// \brief Destructor.
    public: virtual ~ModelResourceRetriever() = default;

    /// \brief Documentation inherited.
    public: bool exists(const dart::common::Uri &_uri) override;

    /// \brief Documentation inherited.
    public: dart::common::ResourcePtr retrieve(
        const dart::common::Uri &_uri) override;

    /// \brief Resolves the model URI into a file path.
    private: dart::common::Uri resolve(const dart::common::Uri &_uri);
  };
  using ModelResourceRetrieverPtr = std::shared_ptr<ModelResourceRetriever>;

  /// \brief Plugin that provides gravity compensation.
  class GAZEBO_VISIBLE GravityCompensationPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: GravityCompensationPlugin();

    /// \brief Destructor.
    public: ~GravityCompensationPlugin();

    /// \brief Init the DART skeleton used to calculate gravity compensation.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the plugin once every iteration of simulation.
    /// \param[in] _info World update information.
    private: void Update(const common::UpdateInfo &_info);

    /// \brief Callback for "~/physics" topic that updates the gravity vector.
    /// \param[in] _msg Physics message.
    private: void OnPhysicsMsg(ConstPhysicsPtr &_msg);

    /// \internal
    /// \brief Private data pointer
    private: std::unique_ptr<GravityCompensationPluginPrivate> dataPtr;
  };
}
#endif
