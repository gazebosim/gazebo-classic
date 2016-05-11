/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <string>

#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/physics/RoadPrivate.hh"
#include "gazebo/physics/Road.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
Road::Road(BasePtr _parent)
: Base(*new RoadPrivate, _parent),
  roadDPtr(static_cast<RoadPrivate*>(this->baseDPtr))
{
}

/////////////////////////////////////////////////
Road::~Road()
{
}

/////////////////////////////////////////////////
void Road::Load(sdf::ElementPtr _elem)
{
  Base::Load(_elem);
  this->SetName(_elem->Get<std::string>("name"));
}

/////////////////////////////////////////////////
void Road::Init()
{
  this->roadDPtr->node = transport::NodePtr(new transport::Node());
  this->roadDPtr->node->Init();

  this->roadDPtr->roadPub =
    this->roadDPtr->node->Advertise<msgs::Road>("~/roads", 10);

  msgs::Road msg;

  msg.set_name(this->Name());

  this->roadDPtr->width = this->roadDPtr->sdf->Get<double>("width");
  msg.set_width(this->roadDPtr->width);

  if (this->roadDPtr->sdf->HasElement("material"))
  {
    sdf::ElementPtr matElem =
        this->roadDPtr->sdf->GetElement("material");
    if (matElem->HasElement("script"))
    {
      sdf::ElementPtr scriptElem = matElem->GetElement("script");
      sdf::ElementPtr uriElem = scriptElem->GetElement("uri");

      // Add all the URI paths to the render engine
      while (uriElem)
      {
        std::string matUri = uriElem->Get<std::string>();
        if (!matUri.empty())
        {
          msg.mutable_material()->mutable_script()->add_uri(matUri);
        }
        uriElem = uriElem->GetNextElement("uri");
      }

      std::string matName = scriptElem->Get<std::string>("name");
      if (!matName.empty())
      {
        msg.mutable_material()->mutable_script()->set_name(matName);
      }
    }
  }
  sdf::ElementPtr pointElem = this->roadDPtr->sdf->GetElement("point");
  while (pointElem)
  {
    ignition::math::Vector3d point = pointElem->Get<ignition::math::Vector3d>();
    pointElem = pointElem->GetNextElement("point");

    msgs::Vector3d *ptMsg = msg.add_point();
    msgs::Set(ptMsg, point);
  }

  this->roadDPtr->roadPub->Publish(msg);
}

/////////////////////////////////////////////////
std::vector<math::Vector3> Road::GetPoints() const
{
  std::vector<math::Vector3> result;
  for (auto pt : this->roadDPtr->points)
    result.push_back(pt);
  return result;
}

/////////////////////////////////////////////////
const std::vector<ignition::math::Vector3d> &Road::Points() const
{
  return this->roadDPtr->points;
}

/////////////////////////////////////////////////
double Road::GetWidth() const
{
  return this->Width();
}

/////////////////////////////////////////////////
double Road::Width() const
{
  return this->roadDPtr->width;
}
