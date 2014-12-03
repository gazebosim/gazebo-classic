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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/common/MeshManager.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/LaserVisualPrivate.hh"
#include "gazebo/rendering/LaserVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
LaserVisual::LaserVisual(const std::string &_name, VisualPtr _vis,
                         const std::string &_topicName)
: Visual(*new LaserVisualPrivate, _name, _vis)
{
  LaserVisualPrivate *dPtr =
      reinterpret_cast<LaserVisualPrivate *>(this->dataPtr);

  dPtr->receivedMsg = false;

  dPtr->node = transport::NodePtr(new transport::Node());
  dPtr->node->Init(dPtr->scene->GetName());

  dPtr->laserScanSub = dPtr->node->Subscribe(_topicName,
      &LaserVisual::OnScan, this);

  dPtr->connection = event::Events::ConnectPreRender(
        boost::bind(&LaserVisual::Update, this));
}

/////////////////////////////////////////////////
LaserVisual::~LaserVisual()
{
  LaserVisualPrivate *dPtr =
      reinterpret_cast<LaserVisualPrivate *>(this->dataPtr);

  for (unsigned int i = 0; i < dPtr->rayFans.size(); ++i)
  {
    this->DeleteDynamicLine(dPtr->rayFans[i]);
    dPtr->rayFans[i] = NULL;
  }
  dPtr->rayFans.clear();
}

/////////////////////////////////////////////////
void LaserVisual::OnScan(ConstLaserScanStampedPtr &_msg)
{
  LaserVisualPrivate *dPtr =
      reinterpret_cast<LaserVisualPrivate *>(this->dataPtr);

  boost::mutex::scoped_lock lock(dPtr->mutex);
  dPtr->laserMsg = _msg;
  dPtr->receivedMsg = true;
}

/////////////////////////////////////////////////
void LaserVisual::Update()
{
  LaserVisualPrivate *dPtr =
      reinterpret_cast<LaserVisualPrivate *>(this->dataPtr);

  boost::mutex::scoped_lock lock(dPtr->mutex);

  // Skip the update if the user is moving the laser.
  if ((this->GetScene()->GetSelectedVisual() &&
      this->GetRootVisual()->GetName() ==
      this->GetScene()->GetSelectedVisual()->GetName()))
  {
    return;
  }

  if (!dPtr->laserMsg || !dPtr->receivedMsg)
    return;

  dPtr->receivedMsg = false;

  double angle = dPtr->laserMsg->scan().angle_min();
  double verticalAngle = dPtr->laserMsg->scan().vertical_angle_min();
  double r;
  math::Vector3 pt;
  math::Pose offset = msgs::Convert(dPtr->laserMsg->scan().world_pose()) -
                      this->GetWorldPose();

  unsigned int vertCount = dPtr->laserMsg->scan().has_vertical_count() ?
      dPtr->laserMsg->scan().vertical_count() : 1u;

  math::Quaternion ray;
  math::Vector3 axis;
  for (unsigned int j = 0; j < vertCount; ++j)
  {
    if (j+1 > dPtr->rayFans.size())
    {
      dPtr->rayFans.push_back(
          this->CreateDynamicLine(rendering::RENDERING_TRIANGLE_FAN));
      dPtr->rayFans[j]->setMaterial("Gazebo/BlueLaser");
      dPtr->rayFans[j]->AddPoint(math::Vector3(0, 0, 0));
      this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
    }
    dPtr->rayFans[j]->SetPoint(0, offset.pos);

    angle = dPtr->laserMsg->scan().angle_min();
    unsigned int count = dPtr->laserMsg->scan().count();
    for (unsigned int i = 0; i < count; ++i)
    {
      r = dPtr->laserMsg->scan().ranges(j*count + i);
      ray.SetFromEuler(math::Vector3(0.0, -verticalAngle, angle));
      axis = offset.rot * ray * math::Vector3(1.0, 0.0, 0.0);
      pt = (axis * r) + offset.pos;

      if (i+1 >= dPtr->rayFans[j]->GetPointCount())
        dPtr->rayFans[j]->AddPoint(pt);
      else
        dPtr->rayFans[j]->SetPoint(i+1, pt);

      angle += dPtr->laserMsg->scan().angle_step();
    }
    verticalAngle += dPtr->laserMsg->scan().vertical_angle_step();
  }
}

/////////////////////////////////////////////////
void LaserVisual::SetEmissive(const common::Color &/*_color*/)
{
}
