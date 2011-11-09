/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Laser Visualization Class
 * Author: Nate Koenig
 * Date: 14 Dec 2007
 */

#include "transport/transport.h"
#include "rendering/Scene.hh"
#include "rendering/DynamicLines.hh"
#include "rendering/LaserVisual.hh"

using namespace gazebo;
using namespace rendering;

/// \brief Constructor
LaserVisual::LaserVisual (const std::string &_name, Scene *_scene,
                          const std::string &_topicName)
 : Visual(_name, _scene), scene(_scene)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_scene->GetName());

  this->laserScanSub = this->node->Subscribe(_topicName, 
      &LaserVisual::OnScan, this);

  this->rayFan = this->CreateDynamicLine(rendering::RENDERING_TRIANGLE_FAN);
  this->rayFanOutline = new DynamicLines(rendering::RENDERING_LINE_STRIP);

  this->rayFan->setMaterial("Gazebo/BlueLaser");
  this->rayFanOutline->setMaterial("Gazebo/Red");

  this->rayFan->AddPoint(math::Vector3(0,0,0));
}
 
LaserVisual::~LaserVisual()
{
  delete this->rayFan;
  delete this->rayFanOutline;

  this->rayFan = NULL;
  this->rayFanOutline = NULL;
}


void LaserVisual::OnScan( const boost::shared_ptr<msgs::LaserScan const> &_msg)
{
  if (!this->GetParent())
  {
    VisualPtr vis = this->scene->GetVisual(_msg->frame());
    vis->AttachVisual(this);
    this->parent = vis;
    std::cout << "Pose[" << this->GetWorldPose() << "]\n";
  }

  double angle = _msg->angle_min();
  double r;
  math::Vector3 pt;
  math::Pose offset = msgs::Convert(_msg->offset());

  for (int i=0; i < _msg->ranges_size(); i++)
  {
    r = _msg->ranges(i) + _msg->range_min();
    pt.x = 0 + r * cos(angle);
    pt.y = 0 + r * sin(angle); 
    pt.z = 0;
    pt += offset.pos;

    if (i+1 >= (int)this->rayFan->GetPointCount())
      this->rayFan->AddPoint(pt);
    else
      this->rayFan->SetPoint(i+1, pt);

    angle += _msg->angle_step();
  }
}
