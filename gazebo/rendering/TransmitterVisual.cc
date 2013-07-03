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

#include "gazebo/transport/transport.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/TransmitterVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
TransmitterVisual::TransmitterVisual(const std::string &_name, VisualPtr _vis,
                         const std::string &_topicName)
: Visual(_name, _vis)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->scene->GetName());

  this->laserScanSub = this->node->Subscribe(_topicName,
      &TransmitterVisual::OnScan, this);

  this->isFirst = true;
  this->receivedMsg = false;

  this->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&TransmitterVisual::Update, this)));
}

/////////////////////////////////////////////////
TransmitterVisual::~TransmitterVisual()
{
  delete this->rayFan;
  this->rayFan = NULL;
}

/////////////////////////////////////////////////
void TransmitterVisual::Load()
{
  Visual::Load();

  this->modelTemplateSDF.reset(new sdf::SDF);
  this->modelTemplateSDF->SetFromString(this->GetTemplateSDFString());
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
    <<     "<material>"
    <<       "<script>"
    <<         "<uri>file://media/materials/scripts/gazebo.material</uri>"
    <<         "<name>Gazebo/GreyTransparent</name>"
    <<       "</script>"
    <<     "</material>"
    <<   "</visual>"
    << "</link>"
    << "<static>true</static>"
    << "</model>"
    << "</sdf>";

  return newModelStr.str();
}

/////////////////////////////////////////////////
void TransmitterVisual::OnScan(ConstPropagationGridPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  this->gridMsg = _msg;
  this->receivedMsg = true;
}

/////////////////////////////////////////////////
void TransmitterVisual::SetEmissive(const common::Color &/*_color*/)
{
}

////////////////////////////////////////////////
void TransmitterVisual::Update()
{
  boost::mutex::scoped_lock lock(this->mutex);

  if (!this->gridMsg || !this->receivedMsg)
    return;

  this->receivedMsg = false;

  if (this->isFirst)
  {
    sdf::ElementPtr visualElem = this->modelTemplateSDF->root
        ->GetElement("model")->GetElement("link")->GetElement("visual");
    for (int i = 0; i < gridMsg->particle_size(); i++)
    {
      std::stringstream si;
      si << i;

      rendering::VisualPtr link2Visual(
          new rendering::Visual("test::" + si.str(), shared_from_this()));
      link2Visual->Load(visualElem);
      link2Visual->SetTransparency(0.9);
      link2Visual->SetPosition(math::Vector3(0, 0, 0));
      this->vectorLink.push_back(link2Visual);
    }
    this->isFirst = false;
  }

  for (int i = 0; i < gridMsg->particle_size(); i++)
  {
    gazebo::msgs::PropagationParticle p;
    p = gridMsg->particle(i);

    double x = p.x();
    double y = p.y();
    
    math::Pose pose = math::Pose(x, y, 0, 0, 0, 0);

    // Assuming that the Rx gain is the same as Tx gain
    double strength = p.signal_level();

    common::Color color(strength, strength, strength);

    rendering::VisualPtr link2Visual = this->vectorLink[i];

    link2Visual->SetPosition(math::Vector3(x, y, 0));
    link2Visual->SetDiffuse(color);
    link2Visual->SetTransparency(0.8);
  }
}
