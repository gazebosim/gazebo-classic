/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <mutex>
#include <vector>

#include <gazebo/common/Time.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>

#include "ShaderParamVisualPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ShaderParamVisualPlugin class.
  class ShaderParamVisualPluginPrivate
  {
    /// \brief Visual whose color and vertices will be changed.
    public: rendering::VisualPtr visual;

    /// \brief A list of params that requires setting sim time.
    /// Each element is a pair of <paramName, shaderType>
    public: std::vector<std::pair<std::string, std::string>> simTimeParams;

    /// \brief Connects to render update event.
    public: event::ConnectionPtr updateConnection;

    /// \brief The current simulation time.
    public: common::Time currentSimTime;

    /// \brief Node used for communication.
    public: transport::NodePtr node;

    /// \brief Subscriber to world info.
    public: transport::SubscriberPtr infoSub;

    /// \brief Mutex to protect sim time updates.
    public: std::mutex mutex;
  };
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(ShaderParamVisualPlugin)

/////////////////////////////////////////////////
ShaderParamVisualPlugin::ShaderParamVisualPlugin()
    : dataPtr(new ShaderParamVisualPluginPrivate)
{
}

/////////////////////////////////////////////////
ShaderParamVisualPlugin::~ShaderParamVisualPlugin()
{
}

/////////////////////////////////////////////////
void ShaderParamVisualPlugin::Load(rendering::VisualPtr _visual,
    sdf::ElementPtr _sdf)
{
  if (!_visual || !_sdf)
  {
    gzerr << "No visual or sdf element specified. Plugin won't load."
          << std::endl;
    return;
  }
  this->dataPtr->visual = _visual;

  if (!_sdf->HasElement("param"))
  {
    gzwarn << "No <param> specified, exiting" << std::endl;
    return;
  }

  // loop and set all shader params
  sdf::ElementPtr paramElem = _sdf->GetElement("param");
  while (paramElem)
  {
    if (!paramElem->HasElement("type") ||
        !paramElem->HasElement("name") ||
        !paramElem->HasElement("value"))
    {
      gzerr << "<param> must have <type>, <name> and <value>" << std::endl;
      paramElem = paramElem->GetNextElement("param");
      continue;
    }
    std::string shaderType = paramElem->Get<std::string>("type");
    std::string paramName = paramElem->Get<std::string>("name");
    std::string value = paramElem->Get<std::string>("value");

    // TIME is reserved keyword for sim time
    if (value == "TIME")
    {
      this->dataPtr->simTimeParams.push_back(
          std::make_pair(paramName, shaderType));
    }
    else
    {
      this->dataPtr->visual->SetMaterialShaderParam(
          paramName, shaderType, value);
    }
    paramElem = paramElem->GetNextElement("param");
  }

  // Connect to the world update signal
  this->dataPtr->updateConnection = event::Events::ConnectPreRender(
      std::bind(&ShaderParamVisualPlugin::Update, this));

  // only subscribe to pose topic to get sim time if a param requests it
  if (!this->dataPtr->simTimeParams.empty())
  {
    this->dataPtr->node = transport::NodePtr(new transport::Node());
    this->dataPtr->node->Init();
    this->dataPtr->infoSub = this->dataPtr->node->Subscribe(
        "~/pose/local/info", &ShaderParamVisualPlugin::OnInfo, this);
  }
}

/////////////////////////////////////////////////
void ShaderParamVisualPlugin::Update()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // set sim time on every render update
  for (auto &p : this->dataPtr->simTimeParams)
  {
    this->dataPtr->visual->SetMaterialShaderParam(p.first, p.second,
      std::to_string(this->dataPtr->currentSimTime.Double()));
  }
}

/////////////////////////////////////////////////
void ShaderParamVisualPlugin::OnInfo(ConstPosesStampedPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // we are only interested in sim time
  this->dataPtr->currentSimTime = msgs::Convert(_msg->time());
}
