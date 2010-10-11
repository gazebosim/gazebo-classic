/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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
  Events::ConnectShowLightsSignal(
      boost::bind(&RenderState::ShowLightsCB,this) );

  Events::ConnectShowJointsSignal(
      boost::bind(&RenderState::ShowJointsCB,this) );

  Events::ConnectShowCamerasSignal(
      boost::bind(&RenderState::ShowCamerasCB,this) );

  Events::ConnectShowContactsSignal(
      boost::bind(&RenderState::ShowContactsCB,this) );

  Events::ConnectShowWireframeSignal(
      boost::bind(&RenderState::ShowWireframeCB,this) );

  Events::ConnectShowBoundingBoxesSignal(
      boost::bind(&RenderState::ShowBoundingBoxesCB,this) );
}

////////////////////////////////////////////////////////////////////////////////
RenderState::~RenderState()
{
  Events::DisconnectShowLightsSignal(
      boost::bind(&RenderState::ShowLightsCB,this) );

  Events::DisconnectShowJointsSignal(
      boost::bind(&RenderState::ShowJointsCB,this) );

  Events::DisconnectShowCamerasSignal(
      boost::bind(&RenderState::ShowCamerasCB,this) );

  Events::DisconnectShowContactsSignal(
      boost::bind(&RenderState::ShowContactsCB,this) );

  Events::DisconnectShowWireframeSignal(
      boost::bind(&RenderState::ShowWireframeCB,this) );

  Events::DisconnectShowBoundingBoxesSignal(
      boost::bind(&RenderState::ShowBoundingBoxesCB,this) );
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
