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

#include <algorithm>
#include <string>
#include <vector>

#include "gazebo/transport/transport.hh"
#include "gazebo/physics/Road.hh"
#include "gazebo/msgs/msgs.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
Road::Road(BasePtr _parent)
  : Base(_parent)
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
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->roadPub = this->node->Advertise<msgs::Road>("~/roads", 10);

  msgs::Road msg;

  msg.set_name(this->GetName());

  this->width = this->sdf->Get<double>("width");
  msg.set_width(this->width);

  if (this->sdf->HasElement("material"))
  {
    sdf::ElementPtr matElem =
        this->sdf->GetElement("material");
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
  sdf::ElementPtr pointElem = this->sdf->GetElement("point");
  while (pointElem)
  {
    ignition::math::Vector3d point = pointElem->Get<ignition::math::Vector3d>();
    pointElem = pointElem->GetNextElement("point");

    msgs::Vector3d *ptMsg = msg.add_point();
    msgs::Set(ptMsg, point);
  }

  this->roadPub->Publish(msg);
}

/////////////////////////////////////////////////
const std::vector<math::Vector3> &Road::GetPoints() const
{
  return this->points;
}

/////////////////////////////////////////////////
double Road::GetWidth() const
{
  return this->width;
}
