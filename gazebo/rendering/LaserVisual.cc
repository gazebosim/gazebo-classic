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

#include <boost/bind.hpp>

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

  dPtr->type = VT_SENSOR;

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

  for (auto ray : dPtr->rayFans)
    this->DeleteDynamicLine(ray);

  for (auto ray : dPtr->noHitRayFans)
    this->DeleteDynamicLine(ray);

  for (auto ray : dPtr->rayLines)
    this->DeleteDynamicLine(ray);

  dPtr->rayFans.clear();
  dPtr->noHitRayFans.clear();
  dPtr->rayLines.clear();
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

  double verticalAngle = dPtr->laserMsg->scan().vertical_angle_min();
  ignition::math::Pose3d offset =
    msgs::ConvertIgn(dPtr->laserMsg->scan().world_pose()) -
    this->GetWorldPose().Ign();

  unsigned int vertCount = dPtr->laserMsg->scan().has_vertical_count() ?
      dPtr->laserMsg->scan().vertical_count() : 1u;

  for (unsigned int j = 0; j < vertCount; ++j)
  {
    if (j+1 > dPtr->rayFans.size())
    {
      dPtr->rayFans.push_back(
          this->CreateDynamicLine(rendering::RENDERING_TRIANGLE_FAN));
      dPtr->rayFans[j]->setMaterial("Gazebo/BlueLaser");
      dPtr->rayFans[j]->AddPoint(math::Vector3(0, 0, 0));

      // No hit ray fans display rays that do not hit obstacles.
      dPtr->noHitRayFans.push_back(
          this->CreateDynamicLine(rendering::RENDERING_TRIANGLE_FAN));
      dPtr->noHitRayFans[j]->setMaterial("Gazebo/LightBlueLaser");
      dPtr->noHitRayFans[j]->AddPoint(math::Vector3(0, 0, 0));

      dPtr->rayLines.push_back(
          this->CreateDynamicLine(rendering::RENDERING_LINE_LIST));
      dPtr->rayLines[j]->setMaterial("Gazebo/BlueLaser");

      this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
    }
    dPtr->rayFans[j]->SetPoint(0, offset.Pos());
    dPtr->noHitRayFans[j]->SetPoint(0, offset.Pos());

    double angle = dPtr->laserMsg->scan().angle_min();
    unsigned int count = dPtr->laserMsg->scan().count();
    for (unsigned int i = 0; i < count; ++i)
    {
      double r = dPtr->laserMsg->scan().ranges(j*count + i);
      ignition::math::Quaterniond ray(
          ignition::math::Vector3d(0.0, -verticalAngle, angle));
      ignition::math::Vector3d axis = offset.Rot() * ray *
        ignition::math::Vector3d(1.0, 0.0, 0.0);

      double hitRange = std::isinf(r) ? 0 : r;
      ignition::math::Vector3d pt = (axis * hitRange) + offset.Pos();

      double noHitRange =
        std::isinf(r) ? dPtr->laserMsg->scan().range_max() : hitRange;
      ignition::math::Vector3d noHitPt = (axis * noHitRange) + offset.Pos();

      // Draw the lines that represent each simulated ray
      if (i >= dPtr->rayLines[j]->GetPointCount()/2)
      {
        dPtr->rayLines[j]->AddPoint(offset.Pos());
        if (std::isinf(r))
          dPtr->rayLines[j]->AddPoint(noHitPt);
        else
          dPtr->rayLines[j]->AddPoint(pt);
      }
      else
      {
        dPtr->rayLines[j]->SetPoint(i*2, offset.Pos());
        if (std::isinf(r))
          dPtr->rayLines[j]->SetPoint(i*2+1, noHitPt);
        else
          dPtr->rayLines[j]->SetPoint(i*2+1, pt);
      }

      // Draw the triangle fan that fill in the gaps for the laser rays
      if (i+1 >= dPtr->rayFans[j]->GetPointCount())
      {
        dPtr->rayFans[j]->AddPoint(pt);
        dPtr->noHitRayFans[j]->AddPoint(noHitPt);
      }
      else
      {
        dPtr->rayFans[j]->SetPoint(i+1, pt);
        dPtr->noHitRayFans[j]->SetPoint(i+1, noHitPt);
      }

      angle += dPtr->laserMsg->scan().angle_step();
    }
    verticalAngle += dPtr->laserMsg->scan().vertical_angle_step();
  }
}

/////////////////////////////////////////////////
void LaserVisual::SetEmissive(const common::Color &/*_color*/)
{
}
