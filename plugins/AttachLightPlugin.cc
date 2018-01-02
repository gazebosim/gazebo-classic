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

#include <map>
#include <mutex>
#include <vector>
#include <ignition/math/Pose3.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/Node.hh"
#include "plugins/AttachLightPlugin.hh"


namespace gazebo
{
  /// \brief Private data class for the AttachLightPlugin class
  class AttachLightPluginPrivate
  {
    /// \brief Event connections
    public: std::vector<event::ConnectionPtr> connections;

    /// \brief Pointer to parent model
    public: physics::ModelPtr model;

    /// \brief Pointer to the world
    public: physics::WorldPtr world;

    /// \brief List of link and light pointers and the light pose offset
    public: std::map<physics::LinkPtr, std::map<physics::LightPtr,
        ignition::math::Pose3d>> linkLights;

    /// \brief Mutex to protect linkLights
    public: std::mutex mutex;

    /// \brief Communication Node
    public: transport::NodePtr node;

    /// \brief Subscribe to the request topic
    public: transport::SubscriberPtr requestSub;
  };
}

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(AttachLightPlugin)

/////////////////////////////////////////////////
AttachLightPlugin::AttachLightPlugin()
    : dataPtr(new AttachLightPluginPrivate)
{
}

/////////////////////////////////////////////////
void AttachLightPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzwarn << "The AttachLightPlugin is being deprecated. Consider using the "
         << "new SDF spec that allows <light> to be added as a child of <link> "
         << "elements" << std::endl;

  this->dataPtr->model = _model;
  this->dataPtr->world = _model->GetWorld();

  if (!_sdf->HasElement("link"))
  {
    gzerr << "No <link> sdf elements found." << std::endl;
    return;
  }

  // load links
  sdf::ElementPtr linkElem = _sdf->GetElement("link");
  while (linkElem)
  {
    std::string linkName;
    if (linkElem->HasElement("link_name"))
    {
      linkName = linkElem->Get<std::string>("link_name");

      physics::LinkPtr link = this->dataPtr->model->GetLink(linkName);
      if (!link)
      {
        gzerr << "Link: '" << linkName << "' not found." << std::endl;
      }
      else
      {
        // load lights
        if (linkElem->HasElement("light"))
        {
          sdf::ElementPtr lightElem = linkElem->GetElement("light");
          while (lightElem)
          {
            std::string lightName = lightElem->Get<std::string>("light_name");
            physics::LightPtr light =
                this->dataPtr->world->LightByName(lightName);

            // lights are loaded before models so we should be able to find them
            if (!light)
            {
              gzerr << "Light: '" << lightName << "' not found." << std::endl;
            }
            else
            {
              // pose is optional
              ignition::math::Pose3d pose;
              if (lightElem->HasElement("pose"))
                pose = lightElem->Get<ignition::math::Pose3d>("pose");

              auto &lights = this->dataPtr->linkLights[link];
              lights[light] = pose;
            }
            lightElem = lightElem->GetNextElement("light");
          }
        }
      }
    }
    linkElem = linkElem->GetNextElement("link");
  }

  if (this->dataPtr->linkLights.empty())
    return;

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  // listen for delete events to remove lens flare if light gets deleted.
  this->dataPtr->requestSub = this->dataPtr->node->Subscribe("~/request",
      &AttachLightPlugin::OnRequest, this);

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateEnd(
      std::bind(&AttachLightPlugin::OnUpdate, this)));
}

/////////////////////////////////////////////////
void AttachLightPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // update light pose based on link pose
  for (auto &it : this->dataPtr->linkLights)
  {
    physics::LinkPtr link = it.first;
    std::map<physics::LightPtr, ignition::math::Pose3d> &lights = it.second;
    for (auto &lightIt : lights)
    {
      physics::LightPtr light = lightIt.first;
      ignition::math::Pose3d pose = lightIt.second;
      light->SetWorldPose(pose + link->WorldPose());
    }
  }
}


//////////////////////////////////////////////////
void AttachLightPlugin::OnRequest(ConstRequestPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (_msg->request() != "entity_delete")
    return;
  for (auto &it : this->dataPtr->linkLights)
  {
    physics::LinkPtr link = it.first;
    std::map<physics::LightPtr, ignition::math::Pose3d> &lights = it.second;
    for (auto lightIt = lights.begin(); lightIt != lights.end(); ++lightIt)
    {
      physics::LightPtr light = lightIt->first;
      if (light->GetScopedName() == _msg->data())
      {
        lights.erase(lightIt);
        return;
      }
    }
  }
}
