/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: Transmitter radiation diagram visualization Class
 * Author: Carlos Agüero
 * Date: 27 Jun 2013
 */

#include "gazebo/common/MeshManager.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/LaserVisual.hh"
#include "gazebo/rendering/TransmitterVisual.hh"

#include "gazebo/sensors/WirelessTransmitter.hh"

#include <iostream>
using namespace std;

using namespace gazebo;
using namespace rendering;
using namespace sensors;

/////////////////////////////////////////////////
TransmitterVisual::TransmitterVisual(const std::string &_name, VisualPtr _vis,
                         const std::string &_topicName)
: Visual(_name, _vis)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->scene->GetName());

  this->laserScanSub = this->node->Subscribe(_topicName,
      &TransmitterVisual::OnScan, this);

  /*this->rayFan = this->CreateDynamicLine(rendering::RENDERING_TRIANGLE_FAN);

  this->rayFan->setMaterial("Gazebo/BlueLaser");
  this->rayFan->AddPoint(math::Vector3(0, 0, 0));
  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);*/

}

/////////////////////////////////////////////////
TransmitterVisual::~TransmitterVisual()
{
  delete this->rayFan;
  this->rayFan = NULL;
}

void TransmitterVisual::Load()
{
  Visual::Load();
  this->modelTemplateSDF.reset(new sdf::SDF);
  this->modelTemplateSDF->SetFromString(this->GetTemplateSDFString());

   sdf::ElementPtr visualElem = this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");

  rendering::VisualPtr linkVisual(new rendering::Visual("test", shared_from_this()));
  linkVisual->Load(visualElem);
  linkVisual->SetTransparency(0.99);
  linkVisual->SetPosition(math::Vector3(0, 0, 0));

  rendering::VisualPtr link2Visual = linkVisual->Clone("New name", linkVisual->GetParent());
  link2Visual->SetPosition(math::Vector3(2, 2, 0.5));

  double x = -10.0;
  double y = -10.0;
  for (int i = 0; i < 21; i++)
  {
    for (int j = 0; j < 21; j++)
    {
      math::Pose pose = math::Pose(x, y, 0, 0, 0, 0);
      double strength = WirelessTransmitter::GetSignalStrength(this->GetWorldPose(), pose);
      common::Color color(strength, strength, strength);

      stringstream sx;
      stringstream sy;
      sx << i;
      sy << j;

      rendering::VisualPtr link2Visual = linkVisual->Clone("New name::" +
        sx.str() + "::" + sy.str(), linkVisual->GetParent());
      
      link2Visual->SetPosition(math::Vector3(x + 0.05, y + 0.05, 0));
      link2Visual->SetDiffuse(color);
      link2Visual->SetTransparency(0.5);

      //cout << sx.str() << "," << sy.str() << ":" << strength << endl;
      x += 1.0;
    }
    x = -10.0;
    y += 1.0;
  }
  

}

/////////////////////////////////////////////////
std::string TransmitterVisual::GetTemplateSDFString()
{
  std::ostringstream newModelStr;
  newModelStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='building_template_model'>"
    << "<pose>0 0 0.0 0 0 0</pose>"
    << "<link name ='link'>"
    <<   "<visual name ='visual'>"
    <<     "<pose>0 0 0.0 0 0 0</pose>"
    <<     "<geometry>"
    <<       "<box>"
    <<         "<size>1 1 1</size>"
    <<       "</box>"
    <<     "</geometry>"
    <<   "</visual>"
    << "</link>"
    << "<static>true</static>"
    << "</model>"
    << "</sdf>";

  return newModelStr.str();
}

/////////////////////////////////////////////////
void TransmitterVisual::OnScan(ConstPosePtr &_msg)
{
  // Skip the update if the user is moving the laser.
  /*if (this->GetScene()->GetSelectedVisual() &&
      this->GetRootVisual()->GetName() ==
      this->GetScene()->GetSelectedVisual()->GetName())
  {
    return;
  }

  math::Vector3 pt = msgs::Convert(_msg->position());
  cout << pt.x << "," << pt.y << endl;

  math::Pose offset = this->GetWorldPose();
  double angle = 0.0;

  this->rayFan->SetPoint(0, offset.pos);
  for (size_t i = 0; i < 20; i++)
  {
    double r = 2.0;
    pt.x = 0 + r * cos(angle);
    pt.y = 0 + r * sin(angle);
    pt.z = 0;
    pt += offset.pos;

    this->rayFan->AddPoint(pt);
    angle += 0.25;
  }*/
}

/////////////////////////////////////////////////
void TransmitterVisual::SetEmissive(const common::Color &/*_color*/)
{
}
