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
/* Desc: RFID Sensor Visualization Class
 * Author:
 * Date:
 */

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/transport/transport.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/common/MeshManager.hh"

#include "gazebo/rendering/RFIDVisualPrivate.hh"
#include "gazebo/rendering/RFIDVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
RFIDVisual::RFIDVisual(const std::string &_name, VisualPtr _vis,
                       const std::string &_topicName)
  : Visual(*new RFIDVisualPrivate, _name, _vis)
{
  RFIDVisualPrivate *dPtr =
      reinterpret_cast<RFIDVisualPrivate *>(this->dataPtr);

  dPtr->node = transport::NodePtr(new transport::Node());
  dPtr->node->Init(dPtr->scene->GetName());

  dPtr->rfidSub = dPtr->node->Subscribe(_topicName, &RFIDVisual::OnScan, this);

  common::MeshManager::Instance()->CreateSphere("rfid_sphere", 5.0, 20, 20);
  this->AttachMesh("rfid_sphere");
  this->SetMaterial("Gazebo/BlueTransparent");
}

/////////////////////////////////////////////////
RFIDVisual::~RFIDVisual()
{
}

/////////////////////////////////////////////////
void RFIDVisual::OnScan(ConstPosePtr &/*_msg*/)
{
  // math::Vector3 pt = msgs::Convert(_msg->position());
  // this->sceneNode->setPosition(Conversions::Convert(pt));
}
