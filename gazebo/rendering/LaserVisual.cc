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
/* Desc: Laser Visualization Class
 * Author: Nate Koenig
 * Date: 14 Dec 2007
 */

#include "gazebo/common/MeshManager.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/LaserVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
LaserVisual::LaserVisual(const std::string &_name, VisualPtr _vis,
                         const std::string &_topicName)
: Visual(_name, _vis)
{
  this->receivedMsg = false;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->scene->GetName());

  this->rayFan = this->CreateDynamicLine(rendering::RENDERING_TRIANGLE_FAN);

  this->rayFan->setMaterial("Gazebo/BlueLaser");
  this->rayFan->AddPoint(math::Vector3(0, 0, 0));
  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  this->laserScanSub = this->node->Subscribe(_topicName,
      &LaserVisual::OnScan, this);

  this->connection = event::Events::ConnectPreRender(
        boost::bind(&LaserVisual::Update, this));
}

/////////////////////////////////////////////////
LaserVisual::~LaserVisual()
{
  if (this->rayFan)
  {
    this->DeleteDynamicLine(this->rayFan);
    this->rayFan = NULL;
  }
}

/////////////////////////////////////////////////
void LaserVisual::OnScan(ConstLaserScanStampedPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->laserMsg = _msg;
  this->receivedMsg = true;
}

/////////////////////////////////////////////////
void LaserVisual::Update()
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Skip the update if the user is moving the laser.
  if (!this->rayFan || (this->GetScene()->GetSelectedVisual() &&
      this->GetRootVisual()->GetName() ==
      this->GetScene()->GetSelectedVisual()->GetName()))
  {
    return;
  }

  if (!this->laserMsg || !this->receivedMsg)
    return;

  this->receivedMsg = false;

  double angle = this->laserMsg->scan().angle_min();
  double r;
  math::Vector3 pt;
  math::Pose offset = msgs::Convert(this->laserMsg->scan().world_pose()) -
                      this->GetWorldPose();

  this->rayFan->SetPoint(0, offset.pos);
  for (size_t i = 0;
       static_cast<int>(i) < this->laserMsg->scan().ranges_size(); ++i)
  {
    r = this->laserMsg->scan().ranges(i);
    pt.x = 0 + r * cos(angle);
    pt.y = 0 + r * sin(angle);
    pt.z = 0;
    pt = offset.rot * pt + offset.pos;

    if (i+1 >= this->rayFan->GetPointCount())
      this->rayFan->AddPoint(pt);
    else
      this->rayFan->SetPoint(i+1, pt);

    angle += this->laserMsg->scan().angle_step();
  }
}

/////////////////////////////////////////////////
void LaserVisual::SetEmissive(const common::Color &/*_color*/)
{
}
