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

#include "gazebo/common/MeshManager.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/GpsVisualPrivate.hh"
#include "gazebo/rendering/GpsVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
GpsVisual::GpsVisual(const std::string &_name, VisualPtr _vis,
                         const std::string &_topicName)
: Visual(*new GpsVisualPrivate, _name, _vis)
{
  GpsVisualPrivate *dPtr =
      reinterpret_cast<GpsVisualPrivate *>(this->dataPtr);
  dPtr->receivedMsg = false;

  // Setup transport
  dPtr->node = transport::NodePtr(new transport::Node());
  dPtr->node->Init(dPtr->scene->GetName());

  // Called when new sensor data arrives
  dPtr->gpsSub = dPtr->node->Subscribe(_topicName,
      &GpsVisual::OnNavigationSolution, this);

  // Called just before rendering takes place
  dPtr->connection = event::Events::ConnectPreRender(
        boost::bind(&GpsVisual::Update, this));
}

/////////////////////////////////////////////////
GpsVisual::~GpsVisual()
{
  GpsVisualPrivate *dPtr =
      reinterpret_cast<GpsVisualPrivate *>(this->dataPtr);
  for (std::map<std::string,DynamicLines*>::iterator iter 
    = dPtr->satLinks.begin(); iter != dPtr->satLinks.end(); ++iter)
    this->DeleteDynamicLine(iter->second);
  dPtr->satLinks.clear();
}

/////////////////////////////////////////////////
void GpsVisual::OnNavigationSolution(ConstGpsPtr &_msg)
{
  GpsVisualPrivate *dPtr =
      reinterpret_cast<GpsVisualPrivate *>(this->dataPtr);
  boost::mutex::scoped_lock lock(dPtr->mutex);
  
  // Save the navigation solution
  dPtr->gpsMsg      = _msg;
  dPtr->receivedMsg = true;

  // Kill off any old satellites
  for (std::map<std::string,DynamicLines*>::iterator iter 
    = dPtr->satLinks.begin(); iter != dPtr->satLinks.end(); ++iter)
    this->DeleteDynamicLine(iter->second);

  // Valid satellite names
  std::map<std::string,DynamicLines*> newLinks;
  for (int i = 0; i <  dPtr->gpsMsg->satellites_size(); ++i)
  {
    // Grab the satellite name
    std::string n = dPtr->gpsMsg->satellites(i).id();

    // Draw a link to this satellite
    if (dPtr->satLinks.find(n) == dPtr->satLinks.end())
      newLinks[n] = this->CreateDynamicLine(rendering::RENDERING_LINE_LIST);

    // Get the receiver and satellite position
    math::Vector3 rxpos = msgs::Convert(dPtr->gpsMsg->world_pos());
    math::Vector3 svpos = msgs::Convert(dPtr->gpsMsg->satellites(i).pos());

    // Configure the satellite
    switch(n.at(0))
    {
      case 'G' : newLinks[n]->setMaterial("Gazebo/Green");  break;  // GPS
      case 'R' : newLinks[n]->setMaterial("Gazebo/Red");    break;  // GLONASS
      case 'J' : newLinks[n]->setMaterial("Gazebo/Orange"); break;  // QZSS
      case 'E' : newLinks[n]->setMaterial("Gazebo/Yellow"); break;  // Galileo
      case 'C' : newLinks[n]->setMaterial("Gazebo/Blue");   break;  // Beidou / Compass
      default  : newLinks[n]->setMaterial("Gazebo/White");  break;  // Other (SBAS?)
    }
    newLinks[n]->AddPoint(rxpos);
    newLinks[n]->AddPoint(svpos);
    newLinks[n]->setVisibilityFlags(GZ_VISIBILITY_GUI);
  }
  
  //Copy over 
  dPtr->satLinks = newLinks;
}

/////////////////////////////////////////////////
void GpsVisual::SetEmissive(const common::Color &/*_color*/)
{
  // Do nothing :D
}