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
/* Desc: A ray
 * Author: Nate Keonig
 * Date: 24 May 2009
 * SVN: $Id:$
 */

#include "Link.hh"
#include "BulletRayCollision.hh"

using namespace gazebo;
using namespace physics;

using namespace physics;

using namespace physics;


//////////////////////////////////////////////////
// Constructor
BulletRayCollision::BulletRayCollision(Link *_body, bool _displayRays)
    : RayCollision<BulletCollision>(body, displayRays)
{
}

//////////////////////////////////////////////////
// Destructor
BulletRayCollision::~BulletRayCollision()
{
}

//////////////////////////////////////////////////
// Update the ray collision
void BulletRayCollision::Update()
{
  RayCollision<BulletCollision>::Update();
}

//////////////////////////////////////////////////
// Set the starting point and direction
void BulletRayCollision::SetPoints(const math::Vector3 &_posStart, const math::Vector3 &_posEnd)
{
  RayCollision<BulletCollision>::SetPoints(_posStart, _posEnd);
}

