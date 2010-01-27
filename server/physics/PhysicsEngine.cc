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
/* Desc: The base class for all physics engines
 * Author: Nate Koenig
 * Date: 11 June 2007
 * SVN: $Id$
 */

#include <boost/thread/recursive_mutex.hpp>

#include "World.hh"
#include "OgreDynamicLines.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "Material.hh"
#include "Shape.hh"
#include "PhysicsEngine.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
PhysicsEngine::PhysicsEngine()
{
  Param::Begin(&this->parameters);
  this->gravityP = new ParamT<Vector3>("gravity",Vector3(0.0, -9.80665, 0.0), 0);
  this->updateRateP = new ParamT<double>("updateRate", 0.0, 0);
  this->stepTimeP = new ParamT<Time>("stepTime",0.025,0);
  Param::End();

  this->mutex = new boost::recursive_mutex();
  this->visual = OgreCreator::Instance()->CreateVisual("Physics_Engine_Visual");
  this->visual->SetVisible(false);
  this->contactLines.resize(100);

  Material *mat = new Material();
  mat->SetName("ContactPointsMaterial");
  mat->SetPointSize(10);
  mat->SetAmbient(Color(1,1,0,1));
  mat->SetDiffuse(Color(1,1,0,1));
  mat->SetEmissive(Color(1,1,0,1));
  std::string matName = OgreCreator::CreateMaterial(mat);

  unsigned int i=0;
  for (this->contactLinesIter = this->contactLines.begin();
       this->contactLinesIter != this->contactLines.end(); 
       this->contactLinesIter++, i++)
  {
    (*this->contactLinesIter) = OgreCreator::Instance()->CreateDynamicLine(
        OgreDynamicRenderable::OT_LINE_LIST);
    (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
    (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
    (*this->contactLinesIter)->setMaterial(matName);
    this->visual->AttachObject(*this->contactLinesIter);
  }

  World::Instance()->ConnectShowContactsSignal( boost::bind(&PhysicsEngine::ShowVisual, this, _1) );

  this->contactLinesIter = this->contactLines.begin();
  delete mat;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
PhysicsEngine::~PhysicsEngine()
{
  if (this->visual)
  {
    OgreCreator::Instance()->DeleteVisual( this->visual );
    this->visual = NULL;
  }

  delete this->gravityP;
  delete this->updateRateP;
  delete this->stepTimeP;

  delete this->mutex;
  this->mutex = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Update the physics engine
void PhysicsEngine::UpdatePhysics()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Return the gavity vector
Vector3 PhysicsEngine::GetGravity() const
{
  return this->gravityP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the time between each update cycle
double PhysicsEngine::GetUpdateRate() const
{
  return this->updateRateP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the time between each update cycle
Time PhysicsEngine::GetStepTime() const
{
  return **this->stepTimeP;
}

////////////////////////////////////////////////////////////////////////////////
/// Lock the physics engine mutex
void PhysicsEngine::LockMutex()
{
  this->mutex->lock();
}

////////////////////////////////////////////////////////////////////////////////
/// Lock the physics engine mutex
void PhysicsEngine::UnlockMutex()
{
  this->mutex->unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Create a geom
Geom *PhysicsEngine::CreateGeom(std::string typeName, Body *body)
{
  for (unsigned int i = 0; i < Shape::TYPE_COUNT; i++)
    if (typeName == Shape::TypeNames[i])
      return this->CreateGeom( (Shape::Type)i, body );

  return NULL; 
}

////////////////////////////////////////////////////////////////////////////////
/// Add a contact visual
void PhysicsEngine::AddContactVisual(Vector3 pos, Vector3 norm)
{
  if (!World::Instance()->GetShowContacts())
    return;

  (*this->contactLinesIter)->SetPoint(0, pos);
  (*this->contactLinesIter)->SetPoint(1, pos+(norm*0.1));

  this->contactLinesIter++;

  if (this->contactLinesIter == this->contactLines.end())
    this->contactLinesIter = this->contactLines.begin();
}

////////////////////////////////////////////////////////////////////////////////
// Set whether to show contacts
void PhysicsEngine::ShowVisual(bool show)
{
  this->visual->SetVisible(show);
  if (show)
    this->contactLinesIter = this->contactLines.begin();
}
