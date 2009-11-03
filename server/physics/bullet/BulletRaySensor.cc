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
/* Desc: Bullet ray sensor
 * Author: Nate Koenig
 * Date: 21 May 2009
 * SVN: $Id:$
 */

#include "Global.hh"
#include "World.hh"

#include "BulletRayGeom.hh"
#include "BulletPhysics.hh"
#include "BulletBody.hh"
#include "BulletRaySensor.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
BulletRaySensor::BulletRaySensor(Body *_body)
  : PhysicsRaySensor(_body)
{
  this->body = dynamic_cast<BulletBody*>(_body);
  this->physicsEngine = dynamic_cast<BulletPhysics*>(World::Instance()->GetPhysicsEngine());

  if (this->body == NULL)
    gzthrow("BulletRaySensor requires an BulletBody");
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
BulletRaySensor::~BulletRaySensor()
{
  std::vector<BulletRayGeom*>::iterator iter;

  for (iter = this->rays.begin(); iter != this->rays.end(); iter++)
  {
    delete (*iter);
  }
  this->rays.clear();
}

////////////////////////////////////////////////////////////////////////////////
/// Add a ray to the sensor
void BulletRaySensor::AddRay(Vector3 start, Vector3 end, double minRange, 
                          double maxRange, bool display)
{
  BulletRayGeom *rayGeom;
  
  rayGeom = (BulletRayGeom*)this->physicsEngine->CreateGeom("ray", this->body );
  rayGeom->SetDisplayRays(display);
  rayGeom->SetMinLength(minRange);
  rayGeom->SetMaxLength(maxRange);

  rayGeom->SetPoints(start,end);
  this->rays.push_back(rayGeom);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of rays
int BulletRaySensor::GetCount() const
{
  return this->rays.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the relative starting and ending points of a ray
void BulletRaySensor::GetRelativePoints(int index, Vector3 &a, Vector3 &b)
{
  if (index <0 || index >= (int)(this->rays.size()))
  {
    std::ostringstream stream;
    stream << "index[" << index << "] is out of range[0-" 
           << this->GetCount() << "]";
    gzthrow(stream.str());
  }

  this->rays[index]->GetRelativePoints(a,b);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the range of a ray
double BulletRaySensor::GetRange(int index) const
{
  if (index <0 || index >= (int)(this->rays.size()))
  {
    std::ostringstream stream;
    stream << "index[" << index << "] is out of range[0-" 
           << this->GetCount() << "]";
    gzthrow(stream.str());
  }

  return this->rays[index]->GetLength();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the retro reflectance value of a ray
double BulletRaySensor::GetRetro(int index) const
{
  if (index <0 || index >= (int)(this->rays.size()))
  {
    std::ostringstream stream;
    stream << "index[" << index << "] is out of range[0-" 
           << this->GetCount() << "]";
    gzthrow(stream.str());
  }

  return this->rays[index]->GetRetro();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the fiducial value of a ray
double BulletRaySensor::GetFiducial(int index) const
{
  if (index <0 || index >= (int)(this->rays.size()))
  {
    std::ostringstream stream;
    stream << "index[" << index << "] is out of range[0-" 
           << this->GetCount() << "]";
    gzthrow(stream.str());
  }

  return this->rays[index]->GetFiducial();
}

////////////////////////////////////////////////////////////////////////////////
/// Update the ray sensor
void BulletRaySensor::Update()
{
  std::vector<BulletRayGeom*>::iterator iter;

  for (iter = this->rays.begin(); iter != this->rays.end(); iter++)
  {
    (*iter)->SetLength( (*iter)->GetMaxLength() );
    (*iter)->SetRetro( 0.0 );
    (*iter)->SetFiducial( -1 );

    // Get the global points of the line
    (*iter)->Update();
  }
}
