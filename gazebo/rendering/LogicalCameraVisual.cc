/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/VisualPrivate.hh"
#include "gazebo/rendering/LogicalCameraVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
LogicalCameraVisual::LogicalCameraVisual(const std::string &_name,
    VisualPtr _vis)
: Visual(_name, _vis)
{
  this->dataPtr->type = VT_SENSOR;
}

/////////////////////////////////////////////////
LogicalCameraVisual::~LogicalCameraVisual()
{
  this->Fini();
}

/////////////////////////////////////////////////
void LogicalCameraVisual::Load(const msgs::LogicalCameraSensor &_msg)
{
  double nearWidth = (tan(_msg.horizontal_fov()*0.5) * _msg.near_clip());
  double nearHeight = nearWidth / _msg.aspect_ratio();

  double farWidth = (tan(_msg.horizontal_fov()*0.5) * _msg.far_clip());
  double farHeight = farWidth / _msg.aspect_ratio();

  DynamicLines *line = this->CreateDynamicLine(RENDERING_LINE_LIST);

  // Draw the near clipping plane as a white rectangle
  line->AddPoint(math::Vector3(_msg.near_clip(), nearWidth, nearHeight));
  line->AddPoint(math::Vector3(_msg.near_clip(), nearWidth, -nearHeight));

  line->AddPoint(math::Vector3(_msg.near_clip(), nearWidth, -nearHeight));
  line->AddPoint(math::Vector3(_msg.near_clip(), -nearWidth, -nearHeight));

  line->AddPoint(math::Vector3(_msg.near_clip(), -nearWidth, -nearHeight));
  line->AddPoint(math::Vector3(_msg.near_clip(), -nearWidth, nearHeight));

  line->AddPoint(math::Vector3(_msg.near_clip(), -nearWidth, nearHeight));
  line->AddPoint(math::Vector3(_msg.near_clip(), nearWidth, nearHeight));

  // Draw the far clipping plane as a white rectangle
  line->AddPoint(math::Vector3(_msg.far_clip(), farWidth, farHeight));
  line->AddPoint(math::Vector3(_msg.far_clip(), farWidth, -farHeight));

  line->AddPoint(math::Vector3(_msg.far_clip(), farWidth, -farHeight));
  line->AddPoint(math::Vector3(_msg.far_clip(), -farWidth, -farHeight));

  line->AddPoint(math::Vector3(_msg.far_clip(), -farWidth, -farHeight));
  line->AddPoint(math::Vector3(_msg.far_clip(), -farWidth, farHeight));

  line->AddPoint(math::Vector3(_msg.far_clip(), -farWidth, farHeight));
  line->AddPoint(math::Vector3(_msg.far_clip(), farWidth, farHeight));

  // Connect the near and far clipping planes with lines
  line->AddPoint(math::Vector3(_msg.near_clip(), nearWidth, nearHeight));
  line->AddPoint(math::Vector3(_msg.far_clip(), farWidth, farHeight));

  line->AddPoint(math::Vector3(_msg.near_clip(), -nearWidth, nearHeight));
  line->AddPoint(math::Vector3(_msg.far_clip(), -farWidth, farHeight));

  line->AddPoint(math::Vector3(_msg.near_clip(), -nearWidth, -nearHeight));
  line->AddPoint(math::Vector3(_msg.far_clip(), -farWidth, -farHeight));

  line->AddPoint(math::Vector3(_msg.near_clip(), nearWidth, -nearHeight));
  line->AddPoint(math::Vector3(_msg.far_clip(), farWidth, -farHeight));

  line->setMaterial("Gazebo/WhiteGlow");
  line->setVisibilityFlags(GZ_VISIBILITY_GUI);

  // Draw green lines from the near clipping plane to the origin
  DynamicLines *sourceLine = this->CreateDynamicLine(RENDERING_LINE_LIST);
  sourceLine->AddPoint(math::Vector3::Zero);
  sourceLine->AddPoint(math::Vector3(_msg.near_clip(), nearWidth, nearHeight));

  sourceLine->AddPoint(math::Vector3::Zero);
  sourceLine->AddPoint(math::Vector3(_msg.near_clip(), -nearWidth, nearHeight));

  sourceLine->AddPoint(math::Vector3::Zero);
  sourceLine->AddPoint(math::Vector3(
        _msg.near_clip(), -nearWidth, -nearHeight));

  sourceLine->AddPoint(math::Vector3::Zero);
  sourceLine->AddPoint(math::Vector3(_msg.near_clip(), nearWidth, -nearHeight));

  sourceLine->setMaterial("Gazebo/PurpleGlow");
  sourceLine->setVisibilityFlags(GZ_VISIBILITY_GUI);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  if (this->dataPtr->parent)
    this->dataPtr->parent->AttachVisual(shared_from_this());
}

/////////////////////////////////////////////////
void LogicalCameraVisual::Fini()
{
  this->DetachObjects();
}
