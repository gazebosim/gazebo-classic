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
/* Desc: Bullet ray sensor
 * Author: Nate Koenig
 * Date: 21 May 2009
 */
/*
#include "World.hh"

#include "BulletPhysics.hh"
#include "BulletRaySensor.hh"
*/

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
BulletRaySensor::BulletRaySensor(Link *_body)
  : PhysicsRaySensor(_body)
{
  this->body = dynamic_cast<BulletLink*>(_body);

  if (this->body == NULL)
    gzthrow("BulletRaySensor requires an BulletLink");
}

//////////////////////////////////////////////////
BulletRaySensor::~BulletRaySensor()
{
  std::vector<BulletRayCollision*>::iterator iter;

  for (iter = this->rays.begin(); iter != this->rays.end(); ++iter)
  {
    delete (*iter);
  }
  this->rays.clear();
}

//////////////////////////////////////////////////
void BulletRaySensor::AddRay(math::Vector3 start, math::Vector3 end,
    double minRange, double maxRange, bool display)
{
  BulletRayCollision *rayCollision;

  rayCollision = static_cast<BulletRayCollision*>(
        this->GetWorld()->CreateCollision("ray", this->body));
  rayCollision->SetDisplayRays(display);
  rayCollision->SetMinLength(minRange);
  rayCollision->SetMaxLength(maxRange);

  rayCollision->SetPoints(start, end);
  this->rays.push_back(rayCollision);
}

//////////////////////////////////////////////////
int BulletRaySensor::GetCount() const
{
  return this->rays.size();
}

//////////////////////////////////////////////////
void BulletRaySensor::GetRelativePoints(int _index,
    math::Vector3 &_a, math::Vector3 &_b)
{
  if (_index <0 || _index >= static_cast<int>(this->rays.size()))
  {
    std::ostringstream stream;
    stream << "_index[" << _index << "] is out of range[0-"
           << this->GetCount() << "]";
    gzthrow(stream.str());
  }

  this->rays[_index]->GetRelativePoints(_a, _b);
}

//////////////////////////////////////////////////
double BulletRaySensor::GetRange(int _index) const
{
  if (_index <0 || _index >= static_cast<int>(this->rays.size()))
  {
    std::ostringstream stream;
    stream << "_index[" << _index << "] is out of range[0-"
           << this->GetCount() << "]";
    gzthrow(stream.str());
  }

  return this->rays[_index]->GetLength();
}

//////////////////////////////////////////////////
double BulletRaySensor::GetRetro(int _index) const
{
  if (_index <0 || _index >= static_cast<int>(this->rays.size()))
  {
    std::ostringstream stream;
    stream << "_index[" << _index << "] is out of range[0-"
           << this->GetCount() << "]";
    gzthrow(stream.str());
  }

  return this->rays[_index]->GetRetro();
}

//////////////////////////////////////////////////
double BulletRaySensor::GetFiducial(int _index) const
{
  if (_index <0 || _index >= static_cast<int>(this->rays.size()))
  {
    std::ostringstream stream;
    stream << "_index[" << _index << "] is out of range[0-"
           << this->GetCount() << "]";
    gzthrow(stream.str());
  }

  return this->rays[_index]->GetFiducial();
}

//////////////////////////////////////////////////
void BulletRaySensor::Update()
{
  std::vector<BulletRayCollision*>::iterator iter;

  for (iter = this->rays.begin(); iter != this->rays.end(); ++iter)
  {
    (*iter)->SetLength((*iter)->GetMaxLength());
    (*iter)->SetRetro(0.0);
    (*iter)->SetFiducial(-1);

    // Get the global points of the line
    (*iter)->Update();
  }
}


