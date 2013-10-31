/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

  this->laserScanSub = this->node->Subscribe(_topicName,
      &LaserVisual::OnScan, this);

  this->connection = event::Events::ConnectPreRender(
        boost::bind(&LaserVisual::Update, this));
}

/////////////////////////////////////////////////
LaserVisual::~LaserVisual()
{
  for (unsigned int i = 0; i < rayFans.size(); ++i)
  {
    this->DeleteDynamicLine(this->rayFans[i]);
    this->rayFans[i] = NULL;
  }
  this->rayFans.clear();
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
  if ((this->GetScene()->GetSelectedVisual() &&
      this->GetRootVisual()->GetName() ==
      this->GetScene()->GetSelectedVisual()->GetName()))
  {
    return;
  }

  if (!this->laserMsg || !this->receivedMsg)
    return;

  this->receivedMsg = false;

  double angle = this->laserMsg->scan().angle_min();
  double verticalAngle = this->laserMsg->scan().vertical_angle_min();
  double r;
  math::Vector3 pt;
  math::Pose offset = msgs::Convert(this->laserMsg->scan().world_pose()) -
                      this->GetWorldPose();

  unsigned int vertCount = this->laserMsg->scan().has_vertical_count() ?
      this->laserMsg->scan().vertical_count() : 1u;

  math::Quaternion ray;
  math::Vector3 axis;
  for (unsigned int j = 0; j < vertCount; ++j)
  {
    if (j+1 > this->rayFans.size())
    {
      this->rayFans.push_back(
        this->CreateDynamicLine(rendering::RENDERING_TRIANGLE_FAN));
      this->rayFans[j]->setMaterial("Gazebo/BlueLaser");
      this->rayFans[j]->AddPoint(math::Vector3(0, 0, 0));
      this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
    }
    this->rayFans[j]->SetPoint(0, offset.pos);

    angle = this->laserMsg->scan().angle_min();
    unsigned int count = this->laserMsg->scan().count();
    for (unsigned int i = 0; i < count; ++i)
    {
      r = this->laserMsg->scan().ranges(j*count + i);
      /*pt.x = r * cos(verticalAngle) * cos(angle);
      pt.y = r * sin(angle);
      pt.z = r * sin(verticalAngle) * cos(angle);
      pt = offset.rot * pt + offset.pos;*/

      ray.SetFromEuler(math::Vector3(0.0, -verticalAngle, angle));
      axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
      pt = (axis * r) + offset.pos;

      if (i+1 >= this->rayFans[j]->GetPointCount())
        this->rayFans[j]->AddPoint(pt);
      else
        this->rayFans[j]->SetPoint(i+1, pt);

      angle += this->laserMsg->scan().angle_step();
    }
    verticalAngle += this->laserMsg->scan().vertical_angle_step();
  }
}

/////////////////////////////////////////////////
void LaserVisual::SetEmissive(const common::Color &/*_color*/)
{
}
