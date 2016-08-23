/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/bind.hpp>

#include "gazebo/transport/transport.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/TransmitterVisualPrivate.hh"
#include "gazebo/rendering/TransmitterVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
TransmitterVisual::TransmitterVisual(const std::string &_name, VisualPtr _vis,
    const std::string &_topicName)
    : Visual(*new TransmitterVisualPrivate, _name, _vis)
{
  TransmitterVisualPrivate *dPtr =
      reinterpret_cast<TransmitterVisualPrivate *>(this->dataPtr);

  dPtr->type = VT_SENSOR;

  dPtr->isFirst = true;
  dPtr->receivedMsg = false;

  dPtr->node = transport::NodePtr(new transport::Node());
  dPtr->node->Init(dPtr->scene->GetName());

  dPtr->points = NULL;

  dPtr->signalPropagationSub = dPtr->node->Subscribe(_topicName,
      &TransmitterVisual::OnNewPropagationGrid, this);

  dPtr->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&TransmitterVisual::Update, this)));
}

/////////////////////////////////////////////////
TransmitterVisual::~TransmitterVisual()
{
  TransmitterVisualPrivate *dPtr =
      reinterpret_cast<TransmitterVisualPrivate *>(this->dataPtr);

  DeleteDynamicLine(dPtr->points);
  dPtr->signalPropagationSub.reset();
}

/////////////////////////////////////////////////
void TransmitterVisual::Load()
{
  Visual::Load();

  TransmitterVisualPrivate *dPtr =
      reinterpret_cast<TransmitterVisualPrivate *>(this->dataPtr);
  dPtr->points = this->CreateDynamicLine(rendering::RENDERING_POINT_LIST);
  dPtr->points->setMaterial("Gazebo/PointCloud");
}

/////////////////////////////////////////////////
void TransmitterVisual::OnNewPropagationGrid(ConstPropagationGridPtr &_msg)
{
  TransmitterVisualPrivate *dPtr =
      reinterpret_cast<TransmitterVisualPrivate *>(this->dataPtr);

  boost::mutex::scoped_lock lock(dPtr->mutex);

  // Just copy the received data but do not update the UI
  dPtr->gridMsg = _msg;
  dPtr->receivedMsg = true;
}

////////////////////////////////////////////////
void TransmitterVisual::Update()
{
  TransmitterVisualPrivate *dPtr =
      reinterpret_cast<TransmitterVisualPrivate *>(this->dataPtr);

  gazebo::msgs::PropagationParticle p;

  boost::mutex::scoped_lock lock(dPtr->mutex);

  if (!dPtr->gridMsg || !dPtr->receivedMsg)
    return;

  // Update the visualization of the last propagation grid received
  dPtr->receivedMsg = false;

  if (dPtr->isFirst)
  {
    for (int i = 0; i < dPtr->gridMsg->particle_size(); ++i)
    {
      p = dPtr->gridMsg->particle(i);
      dPtr->points->AddPoint(p.x(), p.y(), 0.0);
    }
    dPtr->isFirst = false;
  }

  // Update the list of visual elements
  for (int i = 0; i < dPtr->gridMsg->particle_size(); ++i)
  {
    p = dPtr->gridMsg->particle(i);
    dPtr->points->SetPoint(i, math::Vector3(p.x(), p.y(), 0));

    // Crop the signal strength between 0 and 255
    double strength = std::min(std::max(0.0, -p.signal_level()), 255.0);
    // Normalize
    strength = 1.0 - (strength / 255.0);

    // Set the color in gray scale
    common::Color color(strength, strength, strength);
    dPtr->points->SetColor(i, color);
  }
}
