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
/* Desc: common::Parameters for contact joints
 * Author: Nate Koenig
 * Date: 30 July 2003
 */

#include <float.h>
#include "gazebo/physics/ode/ODESurfaceParams.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODESurfaceParams::ODESurfaceParams()
{
}

//////////////////////////////////////////////////
ODESurfaceParams::~ODESurfaceParams()
{
}

//////////////////////////////////////////////////
void ODESurfaceParams::Load(sdf::ElementPtr _sdf)
{
  SurfaceParams::Load(_sdf);
}





