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

#include <algorithm>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/LinkPlot3DPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LinkPlot3DPlugin)

/////////////////////////////////////////////////
LinkPlot3DPlugin::LinkPlot3DPlugin()
{
}

/////////////////////////////////////////////////
LinkPlot3DPlugin::~LinkPlot3DPlugin()
{
}

/////////////////////////////////////////////////
void LinkPlot3DPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "LinkPlot3DPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LinkPlot3DPlugin _sdf pointer is NULL");

  this->world = _model->GetWorld();

  if (!_sdf->HasElement("plot"))
  {
    gzwarn << "No plot elements" << std::endl;
    return;
  }

  // Frequency
  this->frequency = _sdf->Get<int>("frequency", 30).first;

  auto plotElem = _sdf->GetElement("plot");
  while (plotElem)
  {
    auto linkName = plotElem->Get<std::string>("link");

    auto link = _model->GetLink(linkName);

    int id = 0;
    if (link)
    {
      Plot3D plot;

      // Link pointer
      plot.link = link;

      // Relative pose to link
      plot.pose = plotElem->Get<ignition::math::Pose3d>("pose",
          ignition::math::Pose3d::Zero).first;

      // Message
      ignition::msgs::Marker markerMsg;
      markerMsg.set_ns("plot_" + link->GetName());
      markerMsg.set_id(id++);
      markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
//      markerMsg.set_type(ignition::msgs::Marker::POINTS);
      markerMsg.set_type(ignition::msgs::Marker::SPHERE);

      // Material
      auto mat = plotElem->Get<std::string>("material", "Gazebo/Black").first;

      ignition::msgs::Material *matMsg = markerMsg.mutable_material();
      matMsg->mutable_script()->set_name(mat);

      ignition::msgs::Set(markerMsg.mutable_scale(),
          ignition::math::Vector3d(0.01, 0.01, 0.01));

      plot.msg = markerMsg;

      this->plots.push_back(plot);
    }
    else
    {
      gzerr << "Couldn't find link [" << linkName << "] in model [" <<
          _model->GetName() << "]" << std::endl;
    }

    plotElem = plotElem->GetNextElement("plot");
  }

  if (!this->plots.empty())
  {
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&LinkPlot3DPlugin::OnUpdate, this));
  }
}

/////////////////////////////////////////////////
void LinkPlot3DPlugin::OnUpdate()
{
  // Unused callback
  std::function<void(const ignition::msgs::StringMsg &, const bool)> unused =
    [](const ignition::msgs::StringMsg &, const bool &)
  {
  };

  static common::Time prevTime = this->world->GetSimTime();
  auto currentTime = this->world->GetSimTime();

  if ((currentTime - prevTime).Double() < (1.0 / this->frequency))
    return;

  int id = 0;
  for (auto plot : this->plots)
  {
    auto linkWorld = ignition::math::Matrix4d(plot.link->GetWorldPose().Ign());
    auto plotLink = ignition::math::Matrix4d(plot.pose);
    auto plotWorld = linkWorld * plotLink;

    plot.msg.set_id((currentTime.Double()*1000) + id++);

    // POINTS
//    plot.msg.clear_point();
//    ignition::msgs::Set(plot.msg.add_point(), plotWorld.Pose().Pos());

    // SPHERE
    ignition::msgs::Set(plot.msg.mutable_pose(), plotWorld.Pose());

// gzdbg << plot.msg.DebugString() << std::endl;

    this->node.Request("/marker", plot.msg, unused);

    prevTime = currentTime;
  }
}
