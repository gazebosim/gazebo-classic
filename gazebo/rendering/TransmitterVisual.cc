/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/transport/transport.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/TransmitterVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
TransmitterVisual::TransmitterVisual(const std::string &_name, VisualPtr _vis,
    const std::string &_topicName)
: Visual(_name, _vis), isFirst(true), receivedMsg(false)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->scene->GetName());

  this->signalPropagationSub = this->node->Subscribe(_topicName,
      &TransmitterVisual::OnNewPropagationGrid, this);

  this->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&TransmitterVisual::Update, this)));

  this->points = this->CreateDynamicLine(rendering::RENDERING_POINT_LIST);
  this->points->setMaterial("Gazebo/PointCloud");
}

/////////////////////////////////////////////////
TransmitterVisual::~TransmitterVisual()
{
  DeleteDynamicLine(this->points);
  this->signalPropagationSub.reset();
}

/////////////////////////////////////////////////
void TransmitterVisual::Load()
{
  Visual::Load();
}

/////////////////////////////////////////////////
void TransmitterVisual::OnNewPropagationGrid(ConstPropagationGridPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Just copy the received data but do not update the UI
  this->gridMsg = _msg;
  this->receivedMsg = true;
}

////////////////////////////////////////////////
void TransmitterVisual::Update()
{
  gazebo::msgs::PropagationParticle p;

  boost::mutex::scoped_lock lock(this->mutex);

  if (!this->gridMsg || !this->receivedMsg)
    return;

  // Update the visualization of the last propagation grid received
  this->receivedMsg = false;

  if (this->isFirst)
  {
    for (int i = 0; i < gridMsg->particle_size(); ++i)
    {
      p = gridMsg->particle(i);
      this->points->AddPoint(p.x(), p.y(), 0.0);
    }
    this->isFirst = false;
  }

  // Update the list of visual elements
  for (int i = 0; i < gridMsg->particle_size(); ++i)
  {
    p = gridMsg->particle(i);
    this->points->SetPoint(i, math::Vector3(p.x(), p.y(), 0));

    // Crop the signal strength between 0 and 255
    double strength = std::min(std::max(0.0, -p.signal_level()), 255.0);
    // Normalize
    strength = 1.0 - (strength / 255.0);

    // Set the color in gray scale
    common::Color color(strength, strength, strength);
    this->points->SetColor(i, color);
  }
}
