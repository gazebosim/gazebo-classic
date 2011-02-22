/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Render state container
 * Author: Nate Koenig
 * Date: 06 Oct 2010
 */

#include <boost/bind.hpp>

#include "Events.hh"
#include "RenderState.hh"

using namespace gazebo;

bool RenderState::showLights = false;
bool RenderState::showJoints = false;
bool RenderState::showCameras = false;
bool RenderState::showContacts = false;
bool RenderState::showWireframe = false;
bool RenderState::showPhysics = false;
bool RenderState::showBoundingBoxes = false;

RenderState *RenderState::self = NULL;

////////////////////////////////////////////////////////////////////////////////
RenderState::RenderState()
{
  this->connections.push_back( Events::ConnectShowLightsSignal(
      boost::bind(&RenderState::ShowLightsCB,this) ) );

  this->connections.push_back( Events::ConnectShowJointsSignal(
      boost::bind(&RenderState::ShowJointsCB,this) ) );

  this->connections.push_back( Events::ConnectShowCamerasSignal(
      boost::bind(&RenderState::ShowCamerasCB,this) ) );

  this->connections.push_back( Events::ConnectShowContactsSignal(
      boost::bind(&RenderState::ShowContactsCB,this) ) );

  this->connections.push_back( Events::ConnectShowWireframeSignal(
      boost::bind(&RenderState::ShowWireframeCB,this) ) );

  this->connections.push_back( Events::ConnectShowBoundingBoxesSignal(
      boost::bind(&RenderState::ShowBoundingBoxesCB,this) ) );
}

////////////////////////////////////////////////////////////////////////////////
RenderState::~RenderState()
{
  this->connections.clear();
}

////////////////////////////////////////////////////////////////////////////////
void RenderState::Init()
{
  if (self == NULL)
    self = new RenderState();
}

////////////////////////////////////////////////////////////////////////////////
bool RenderState::GetShowLights()
{
  return showLights;
}

////////////////////////////////////////////////////////////////////////////////
bool RenderState::GetShowJoints()
{
  return showJoints;
}

////////////////////////////////////////////////////////////////////////////////
bool RenderState::GetShowCameras() 
{
  return showCameras;
}

////////////////////////////////////////////////////////////////////////////////
bool RenderState::GetShowContacts() 
{
  return showContacts;
}

////////////////////////////////////////////////////////////////////////////////
bool RenderState::GetShowWireframe() 
{
  return showWireframe;
}

////////////////////////////////////////////////////////////////////////////////
bool RenderState::GetShowPhysics() 
{
  return showPhysics;
}

////////////////////////////////////////////////////////////////////////////////
bool RenderState::GetShowBoundingBoxes() 
{
  return showBoundingBoxes;
}

////////////////////////////////////////////////////////////////////////////////
void RenderState::ShowLightsCB()
{
  this->showLights = !this->showLights;
}

////////////////////////////////////////////////////////////////////////////////
void RenderState::ShowJointsCB()
{
  this->showJoints = !this->showJoints;
}

////////////////////////////////////////////////////////////////////////////////
void RenderState::ShowCamerasCB()
{
  this->showCameras = !this->showCameras;
}

////////////////////////////////////////////////////////////////////////////////
void RenderState::ShowContactsCB()
{
  this->showContacts = !this->showContacts;
}

////////////////////////////////////////////////////////////////////////////////
void RenderState::ShowWireframeCB()
{
  this->showWireframe = !this->showWireframe;
}

////////////////////////////////////////////////////////////////////////////////
void RenderState::ShowPhysicsCB()
{
  this->showPhysics = !this->showPhysics;
}

////////////////////////////////////////////////////////////////////////////////
void RenderState::ShowBoundingBoxesCB()
{
  this->showBoundingBoxes = !this->showBoundingBoxes;
}
