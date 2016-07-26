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
  dPtr->node->Init(dPtr->scene->Name());

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

  for (auto ray : dPtr->rayStrips)
    this->DeleteDynamicLine(ray);

  for (auto ray : dPtr->noHitRayStrips)
    this->DeleteDynamicLine(ray);

  for (auto ray : dPtr->deadzoneRayFans)
    this->DeleteDynamicLine(ray);

  for (auto ray : dPtr->rayLines)
    this->DeleteDynamicLine(ray);

  dPtr->rayStrips.clear();
  dPtr->noHitRayStrips.clear();
  dPtr->deadzoneRayFans.clear();
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
  if ((this->GetScene()->SelectedVisual() &&
      this->GetRootVisual()->GetName() ==
      this->GetScene()->SelectedVisual()->GetName()))
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

  double minRange = dPtr->laserMsg->scan().range_min();

  // Process each ray fan
  for (unsigned int j = 0; j < vertCount; ++j)
  {
    // Create a new render objects, if there are not enough already allocated.
    if (j+1 > dPtr->rayStrips.size())
    {
      // Ray strips fill in-between the ray lines in areas that have
      // intersected an object.
      dPtr->rayStrips.push_back(
          this->CreateDynamicLine(rendering::RENDERING_TRIANGLE_STRIP));
      dPtr->rayStrips[j]->setMaterial("Gazebo/BlueLaser");

      // No hit ray strips fill in-between the ray lines in areas that have
      // not intersected an object.
      dPtr->noHitRayStrips.push_back(
          this->CreateDynamicLine(rendering::RENDERING_TRIANGLE_STRIP));
      dPtr->noHitRayStrips[j]->setMaterial("Gazebo/LightBlueLaser");

      // Deadzone ray fans display areas that are between the sensor's origin
      // and start of the rays.
      dPtr->deadzoneRayFans.push_back(
          this->CreateDynamicLine(rendering::RENDERING_TRIANGLE_FAN));
      dPtr->deadzoneRayFans[j]->setMaterial("Gazebo/BlackTransparent");
      dPtr->deadzoneRayFans[j]->AddPoint(ignition::math::Vector3d(0, 0, 0));

      // Individual ray lines
      dPtr->rayLines.push_back(
          this->CreateDynamicLine(rendering::RENDERING_LINE_LIST));
      dPtr->rayLines[j]->setMaterial("Gazebo/BlueLaser");

      this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
    }
    dPtr->deadzoneRayFans[j]->SetPoint(0, offset.Pos());

    double angle = dPtr->laserMsg->scan().angle_min();
    unsigned int count = dPtr->laserMsg->scan().count();

    // Process each ray in the current scan.
    for (unsigned int i = 0; i < count; ++i)
    {
      // Calculate the range of the ray
      double r = dPtr->laserMsg->scan().ranges(j*count + i);
      bool inf = std::isinf(r);

      ignition::math::Quaterniond ray(
          ignition::math::Vector3d(0.0, -verticalAngle, angle));

      ignition::math::Vector3d axis = offset.Rot() * ray *
        ignition::math::Vector3d(1.0, 0.0, 0.0);

      // Check for infinite range, which indicates the ray did not
      // intersect an object.
      double hitRange = inf ? 0 : r;

      // Compute the start point of the ray
      ignition::math::Vector3d startPt = (axis * minRange) + offset.Pos();

      // Compute the end point of the ray
      ignition::math::Vector3d pt = (axis * hitRange) + offset.Pos();

      double noHitRange = inf ? dPtr->laserMsg->scan().range_max() : hitRange;

      // Compute the end point of the no-hit ray
      ignition::math::Vector3d noHitPt = (axis * noHitRange) + offset.Pos();

      // Draw the lines and strips that represent each simulated ray
      if (i >= dPtr->rayLines[j]->GetPointCount()/2)
      {
        dPtr->rayLines[j]->AddPoint(startPt);
        dPtr->rayLines[j]->AddPoint(inf ? noHitPt : pt);

        dPtr->rayStrips[j]->AddPoint(startPt);
        dPtr->rayStrips[j]->AddPoint(inf ? startPt : pt);

        dPtr->noHitRayStrips[j]->AddPoint(startPt);
        dPtr->noHitRayStrips[j]->AddPoint(inf ? noHitPt : pt);
      }
      else
      {
        dPtr->rayLines[j]->SetPoint(i*2, startPt);
        dPtr->rayLines[j]->SetPoint(i*2+1, inf ? noHitPt : pt);

        dPtr->rayStrips[j]->SetPoint(i*2, startPt);
        dPtr->rayStrips[j]->SetPoint(i*2+1, inf ? startPt : pt);

        dPtr->noHitRayStrips[j]->SetPoint(i*2, startPt);
        dPtr->noHitRayStrips[j]->SetPoint(i*2+1, inf ? noHitPt : pt);
      }

      // Draw the triangle fan that indicates the dead zone.
      if (i+1 >= dPtr->deadzoneRayFans[j]->GetPointCount())
        dPtr->deadzoneRayFans[j]->AddPoint(startPt);
      else
        dPtr->deadzoneRayFans[j]->SetPoint(i+1, startPt);

      angle += dPtr->laserMsg->scan().angle_step();
    }
    verticalAngle += dPtr->laserMsg->scan().vertical_angle_step();
  }
}

/////////////////////////////////////////////////
void LaserVisual::SetEmissive(const common::Color &/*_color*/,
    const bool /*_cascade*/)
{
}
