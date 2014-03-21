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

  this->texture = this->sdf->Get<std::string>("texture");
  
  std::transform(this->texture.begin(), this->texture.end(), this->texture.begin(), ::tolower);

  msg.set_texture(this->texture);



//  if (this->texture == "footway")
//   
//     msg.set_texture(msgs::Road::FOOTWAY);
//    
//  else if (this->texture == "pedestrian")
//   
//     msg.set_texture(msgs::Road::PEDESTRIAN);
//
//  else if (this->texture == "motorway")
//  
//    msg.set_texture(msgs::Road::MOTORWAY);
//
//  else if (this->texture == "lanes_6")
//  
//    msg.set_texture(msgs::Road::LANES_6);
//
//  else if (this->texture == "trunk")
//   
//     msg.set_texture(msgs::Road::TRUNK);
//
//  else if (this->texture == "lanes_4")
//   
//     msg.set_texture(msgs::Road::LANES_4);
//
//  else if (this->texture == "primary")
//   
//     msg.set_texture(msgs::Road::PRIMARY);
//
//  else if (this->texture == "secondary")
//   
//     msg.set_texture(msgs::Road::SECONDARY);
//
//  else if (this->texture == "tertiary")
//   
//     msg.set_texture(msgs::Road::TERTIARY);
//
//  else if (this->texture == "lanes_2")
//   
//     msg.set_texture(msgs::Road::LANES_2);
//
//  else if (this->texture == "residential")
//   
//     msg.set_texture(msgs::Road::RESIDENTIAL);
//

 
  sdf::ElementPtr pointElem = this->sdf->GetElement("point");
  while (pointElem)
  {
    math::Vector3 point = pointElem->Get<math::Vector3>();
    pointElem = pointElem->GetNextElement("point");

    msgs::Vector3d *ptMsg = msg.add_point();
    msgs::Set(ptMsg, point);
  }

  this->roadPub->Publish(msg);
}
