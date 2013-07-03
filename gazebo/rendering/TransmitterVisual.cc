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
      &TransmitterVisual::OnNewPropagationGrid, this);

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
void TransmitterVisual::OnNewPropagationGrid(ConstPropagationGridPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Just copy the data
  this->gridMsg = _msg;
  this->receivedMsg = true;
}

////////////////////////////////////////////////
void TransmitterVisual::Update()
{
  boost::mutex::scoped_lock lock(this->mutex);

  if (!this->gridMsg || !this->receivedMsg)
    return;

  // Update the visualization of the last propagation grid received
  this->receivedMsg = false;

  if (this->isFirst)
  {
    // Allocate the grid of visual elements
    sdf::ElementPtr visualElem = this->modelTemplateSDF->root
        ->GetElement("model")->GetElement("link")->GetElement("visual");

    for (int i = 0; i < gridMsg->particle_size(); i++)
    {
      std::stringstream si;
      si << i;

      rendering::VisualPtr linkVisual(
          new rendering::Visual("test::" + si.str(), shared_from_this()));
      linkVisual->Load(visualElem);
      linkVisual->SetTransparency(0.9);
      linkVisual->SetPosition(math::Vector3(0, 0, 0));
      this->vectorLink.push_back(linkVisual);
    }
    this->isFirst = false;
  }

  // Update the list of visual elements
  for (int i = 0; i < gridMsg->particle_size(); i++)
  {
    gazebo::msgs::PropagationParticle p;
    p = gridMsg->particle(i);

    math::Pose pose = math::Pose(p.x(), p.y(), 0, 0, 0, 0);

    // Assuming that the Rx gain is the same as Tx gain
    double strength = p.signal_level();
    common::Color color(strength, strength, strength);

    rendering::VisualPtr linkVisual = this->vectorLink[i];
    linkVisual->SetPosition(math::Vector3(p.x(), p.y(), 0));
    linkVisual->SetDiffuse(color);
    linkVisual->SetTransparency(0.8);
  }
}
