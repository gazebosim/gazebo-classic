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

#include "RenderState.hh"
#include "Events.hh"
#include "World.hh"
#include "OgreVisual.hh"
#include "OgreDynamicLines.hh"
#include "OgreCreator.hh"
#include "Material.hh"
#include "Shape.hh"
#include "PhysicsEngine.hh"
#include "Simulator.hh"  // disable X

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
PhysicsEngine::PhysicsEngine()
{
  Param::Begin(&this->parameters);
  this->gravityP = new ParamT<Vector3>("gravity",Vector3(0.0, -9.80665, 0.0), 0);
  this->gravityP->Callback(&PhysicsEngine::SetGravity, this);

  this->updateRateP = new ParamT<double>("updateRate", 0.0, 0);
  this->stepTimeP = new ParamT<Time>("stepTime",0.025,0);
  Param::End();

  this->mutex = new boost::recursive_mutex();
  this->locked = false;

  if (Simulator::Instance()->GetRenderEngineEnabled())
  {
    this->visual = OgreCreator::Instance()->CreateVisual("Physics_Engine_Visual");
    this->visual->SetVisible(false);
    this->visual->SetCastShadows(false);
    this->visual->SetUseRTShader(false);
    this->contactLines.resize(5000);

    Material *mat = new Material();
    mat->SetName("ContactPointsMaterial");
    mat->SetPointSize(10);
    mat->SetAmbient(Color(1,0,0,1));
    mat->SetDiffuse(Color(1,0,0,1));
    mat->SetEmissive(Color(1,0,0,1));
    std::string matName = OgreCreator::CreateMaterial(mat);

    unsigned int i=0;
    for (this->contactLinesIter = this->contactLines.begin();
         this->contactLinesIter != this->contactLines.end(); 
         this->contactLinesIter++, i++)
    {
      (*this->contactLinesIter) = this->visual->AddDynamicLine(
          OgreDynamicRenderable::OT_LINE_LIST);
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(Vector3(0,0,0));
      (*this->contactLinesIter)->setMaterial(matName);
    }

    Events::ConnectShowContactsSignal( boost::bind(&PhysicsEngine::ToggleShowVisual, this) );

    this->contactLinesIter = this->contactLines.begin();
    delete mat;
  }

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
PhysicsEngine::~PhysicsEngine()
{
  Events::DisconnectShowContactsSignal( boost::bind(&PhysicsEngine::ToggleShowVisual, this) );

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
/// Set the time between each update cycle
void PhysicsEngine::SetUpdateRate(double rate) const
{
  this->updateRateP->SetValue(rate);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the time between each update cycle
Time PhysicsEngine::GetStepTime() const
{
  return **this->stepTimeP;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the time between each update cycle
void PhysicsEngine::SetStepTime(Time time)
{
  this->stepTimeP->SetValue(time);
}

////////////////////////////////////////////////////////////////////////////////
/// Lock the physics engine mutex
void PhysicsEngine::LockMutex()
{
  this->mutex->lock();
  this->locked = true;
}

////////////////////////////////////////////////////////////////////////////////
/// Lock the physics engine mutex
void PhysicsEngine::UnlockMutex()
{
  this->mutex->unlock();
  this->locked = false;
}

////////////////////////////////////////////////////////////////////////////////
/// Add a contact visual
void PhysicsEngine::AddContactVisual(Vector3 pos, Vector3 norm)
{
  if (!RenderState::GetShowContacts())
    return;

  Vector3 e1 = norm.GetPerpendicular(); e1.Normalize();
  Vector3 e2 = norm.GetCrossProd(e1); e2.Normalize();

  (*this->contactLinesIter)->SetPoint( 0, pos);
  (*this->contactLinesIter)->SetPoint( 1, pos+(norm*0.2)+(e1*0.05)+(e2*0.05));

  (*this->contactLinesIter)->SetPoint( 2, pos);
  (*this->contactLinesIter)->SetPoint( 3, pos+(norm*0.2)+(e1*0.05)-(e2*0.05));

  (*this->contactLinesIter)->SetPoint( 4, pos);
  (*this->contactLinesIter)->SetPoint( 5, pos+(norm*0.2)-(e1*0.05)+(e2*0.05));

  (*this->contactLinesIter)->SetPoint( 6, pos);
  (*this->contactLinesIter)->SetPoint( 7, pos+(norm*0.2)-(e1*0.05)-(e2*0.05));

  (*this->contactLinesIter)->SetPoint( 8, pos+(norm*0.2)+(e1*0.05)+(e2*0.05));
  (*this->contactLinesIter)->SetPoint( 9, pos+(norm*0.2)-(e1*0.05)+(e2*0.05));

  (*this->contactLinesIter)->SetPoint(10, pos+(norm*0.2)-(e1*0.05)+(e2*0.05));
  (*this->contactLinesIter)->SetPoint(11, pos+(norm*0.2)-(e1*0.05)-(e2*0.05));

  (*this->contactLinesIter)->SetPoint(12, pos+(norm*0.2)-(e1*0.05)-(e2*0.05));
  (*this->contactLinesIter)->SetPoint(13, pos+(norm*0.2)+(e1*0.05)-(e2*0.05));

  (*this->contactLinesIter)->SetPoint(14, pos+(norm*0.2)+(e1*0.05)-(e2*0.05));
  (*this->contactLinesIter)->SetPoint(15, pos+(norm*0.2)+(e1*0.05)+(e2*0.05));

  this->contactLinesIter++;

  if (this->contactLinesIter == this->contactLines.end())
    this->contactLinesIter = this->contactLines.begin();
}

////////////////////////////////////////////////////////////////////////////////
// Toggle whether to show contacts
void PhysicsEngine::ToggleShowVisual()
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;
  this->visual->ToggleVisible();
  this->contactLinesIter = this->contactLines.begin();
}

////////////////////////////////////////////////////////////////////////////////
// Set whether to show contacts
void PhysicsEngine::ShowVisual(bool show)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;
  this->visual->SetVisible(show);
  if (show)
    this->contactLinesIter = this->contactLines.begin();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the count of the parameters
unsigned int PhysicsEngine::GetParamCount() const
{
  return this->parameters.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a param by index
Param *PhysicsEngine::GetParam(unsigned int index) const
{
  if (index < this->parameters.size())
    return this->parameters[index];
  else
    gzerr(0) << "Invalid index[" << index << "]\n";
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a parameter by name
Param *PhysicsEngine::GetParam(const std::string &key) const
{
  std::vector<Param*>::const_iterator iter;
  Param *result = NULL;

  for (iter = this->parameters.begin(); iter != this->parameters.end(); iter++)
  {
    if ((*iter)->GetKey() == key)
    {
      result = *iter;
      break;
    }
  }

  if (result == NULL)
    gzerr(0) << "Unable to find Param using key[" << key << "]\n";

  return result;

}

////////////////////////////////////////////////////////////////////////////////
/// Set a parameter by name
void PhysicsEngine::SetParam(const std::string &key, const std::string &value)
{
  std::vector<Param*>::const_iterator iter;
  Param *result = NULL;

  for (iter = this->parameters.begin(); iter != this->parameters.end(); iter++)
  {
    if ((*iter)->GetKey() == key)
    {
      result = *iter;
      break;
    }
  }

  if (result == NULL)
    gzerr(0) << "Unable to find Param using key[" << key << "]\n";
  else
    result->SetFromString( value, true );
}
