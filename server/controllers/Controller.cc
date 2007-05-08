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
/*
 * Desc: Controller base class.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id$
 */

#include "GazeboError.hh"
#include "XMLConfig.hh"
#include "Controller.hh"
#include "Model.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Controller::Controller()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Controller::~Controller()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Load the controller. Called once on startup
int Controller::Load(XMLConfigNode *node)
{
  if (!this->model)
    throw GazeboError("Controller::Load","model has not been set");

  this->updatePeriod = 1.0 / (node->GetDouble("updateRate", 10) + 1e-6);

  return this->LoadChild(node);
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize the controller. Called once on startup.
int Controller::Init()
{
  return this->InitChild();
}

////////////////////////////////////////////////////////////////////////////////
/// Update the controller. Called every cycle.
int Controller::Update()
{
  return this->UpdateChild();
}

////////////////////////////////////////////////////////////////////////////////
/// Finialize the controller. Called once on completion.
int Controller::Fini()
{
  return this->FiniChild();
}

////////////////////////////////////////////////////////////////////////////////
// Set the model for the controller
void Controller::SetModel(Model *model)
{
  this->model = model;
}
