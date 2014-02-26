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
/* Desc: RFID Visualization Class
 * Author: Jonas Mellin & Zakiruz Zaman
 * Date: 6th December 2011
 */

#include "gazebo/transport/transport.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/common/MeshManager.hh"
#include "gazebo/rendering/RFIDTagVisualPrivate.hh"
#include "gazebo/rendering/RFIDTagVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
RFIDTagVisual::RFIDTagVisual(const std::string &_name, VisualPtr _vis,
                             const std::string &_topicName)
  : Visual(*new RFIDTagVisualPrivate, _name, _vis)
{
  RFIDTagVisualPrivate *dPtr =
      reinterpret_cast<RFIDTagVisualPrivate *>(this->dataPtr);

  dPtr->node = transport::NodePtr(new transport::Node());
  dPtr->node->Init(dPtr->scene->GetName());

  dPtr->rfidSub = dPtr->node->Subscribe(_topicName,
      &RFIDTagVisual::OnScan, this);

  common::MeshManager::Instance()->CreateSphere("contact_sphere", .2, 10, 10);

  this->AttachMesh("contact_sphere");
  this->SetMaterial("Gazebo/OrangeTransparent");
}

/////////////////////////////////////////////////
RFIDTagVisual::~RFIDTagVisual()
{
}

/////////////////////////////////////////////////
void RFIDTagVisual::OnScan(ConstPosePtr &/*_msg*/)
{
  // math::Vector3 pt = msgs::Convert(_msg->position());
  // this->sceneNode->setPosition(Conversions::Convert(pt));
}
