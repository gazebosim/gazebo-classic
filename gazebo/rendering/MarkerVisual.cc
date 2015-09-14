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

#include "gazebo/common/Console.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/MarkerVisualPrivate.hh"
#include "gazebo/rendering/MarkerVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
MarkerVisual::MarkerVisual(const std::string &_name, VisualPtr _vis)
: Visual(*new MarkerVisualPrivate, _name, _vis)
{
  this->dPtr = reinterpret_cast<MarkerVisualPrivate *>(this->dataPtr);
}

/////////////////////////////////////////////////
MarkerVisual::~MarkerVisual()
{
}

/////////////////////////////////////////////////
void MarkerVisual::Load(const msgs::Marker &_msg)
{
 std::lock_guard<std::mutex> lock(this->dPtr->mutex);

 if (_msg.action() == msgs::Marker::ADD_MODIFY)
 {
   this->AddModify(_msg);
 }
}

/////////////////////////////////////////////////
void MarkerVisual::AddModify(const msgs::Marker &_msg)
{
  // Set the type of visual
  if (this->dPtr->msg.type() != _msg.type())
  {
    this->DetachObjects();

    switch(_msg.type())
    {
      case msgs::Marker::CUBE:
        this->AttachMesh("unit_box");
        break;
      case msgs::Marker::CYLINDER:
        this->AttachMesh("unit_cylinder");
        break;
      case msgs::Marker::LINE_STRIP:
      case msgs::Marker::LINE_LIST:
      case msgs::Marker::POINTS:
        this->DynamicRenderable(_msg);
        break;
      case msgs::Marker::SPHERE:
        this->AttachMesh("unit_sphere");
        break;
      case msgs::Marker::TEXT:
        this->Text(_msg);
        break;
      case msgs::Marker::TRIANGLE_FAN:
      case msgs::Marker::TRIANGLE_LIST:
        this->Triangle(_msg);
        break;
      default:
        gzerr << "Unable to create marker of type[" << _msg.type() << "]\n";
        break;
    };
  }

  // Set the marker's material
  if (_msg.has_material())
  {
    this->ProcessMaterialMsg(_msg.material());
  }

  // Scale the visual
  if (_msg.has_scale())
    this->SetScale(msgs::ConvertIgn(_msg.scale()));

  // Set the visual's pose
  if (_msg.has_pose())
    this->SetPose(msgs::ConvertIgn(_msg.pose()));
}

/////////////////////////////////////////////////
void MarkerVisual::DynamicRenderable(const msgs::Marker &_msg)
{
  if (this->dPtr->dynamicRenderable == NULL)
  {
    switch (_msg.type())
    {
      case msgs::Marker::LINE_STRIP:
        this->dPtr->dynamicRenderable =
          this->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
        break;
      case msgs::Marker::LINE_LIST:
        this->dPtr->dynamicRenderable =
          this->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
        break;
      case msgs::Marker::POINTS:
        this->dPtr->dynamicRenderable =
          this->CreateDynamicLine(rendering::RENDERING_POINT_LIST);
        break;
      default:
        gzerr << "Unable to create dynamic renderable of type[" <<
          _msg.type() << "]\n";
        break;
    };

    for (int i = 0; i < _msg.point_size(); ++i)
      this->dPtr->dynamicRenderable->AddPoint(msgs::ConvertIgn(_msg.point(i)));
  }
  else
  {
    for (int i = 0; i < _msg.point_size(); ++i)
    {
      this->dPtr->dynamicRenderable->SetPoint(i,
          msgs::ConvertIgn(_msg.point(i)));
    }
  }
}


/////////////////////////////////////////////////
void MarkerVisual::Triangle(const msgs::Marker & /*_msg*/)
{
}

/////////////////////////////////////////////////
void MarkerVisual::Text(const msgs::Marker & /*_msg*/)
{
}
