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
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/gui/EntityMaker.hh"

using namespace gazebo;
using namespace gui;

bool EntityMaker::snapToGrid = true;
double EntityMaker::snapDistance = 0.4;
double EntityMaker::snapGridSize = 1.0;

//////////////////////////////////////////////////
EntityMaker::EntityMaker()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->visPub = this->node->Advertise<msgs::Visual>("~/visual");
  this->makerPub = this->node->Advertise<msgs::Factory>("~/factory");
  this->requestPub = this->node->Advertise<msgs::Request>("~/request");
}

//////////////////////////////////////////////////
EntityMaker::~EntityMaker()
{
  this->camera.reset();
  this->node->Fini();
  this->node.reset();
  this->visPub.reset();
  this->requestPub.reset();
}

//////////////////////////////////////////////////
void EntityMaker::SetSnapToGrid(bool _snap)
{
  snapToGrid = _snap;
}


//////////////////////////////////////////////////
void EntityMaker::OnMousePush(const common::MouseEvent &/*event*/)
{
}

//////////////////////////////////////////////////
void EntityMaker::OnMouseRelease(const common::MouseEvent &/*event*/)
{
}

//////////////////////////////////////////////////
void EntityMaker::OnMouseDrag(const common::MouseEvent &/*event*/)
{
}

//////////////////////////////////////////////////
void EntityMaker::OnMouseMove(const common::MouseEvent &/*_event*/)
{
}

//////////////////////////////////////////////////
ignition::math::Vector3d EntityMaker::GetSnappedPoint(
    ignition::math::Vector3d _p)
{
  ignition::math::Vector3d result = _p;

  if (this->snapToGrid)
  {
    ignition::math::Vector3d rounded = (_p / this->snapGridSize).Rounded() *
      this->snapGridSize;
    if (fabs(_p.X() - rounded.X()) < this->snapDistance)
      result.X() = rounded.X();
    if (fabs(_p.Y() - rounded.Y()) < this->snapDistance)
      result.Y() = rounded.Y();
  }

  return result;
}
