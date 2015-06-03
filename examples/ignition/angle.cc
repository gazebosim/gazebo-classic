/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <gazebo/math/Angle.hh>
#include <ignition/math/Angle.hh>

/////////////////////////////////////////////////
// Migration from gazebo::math::Angle to ignition::math::Angle examples
// See Ignition documentation http://ignitionrobotics.org/libraries/math
int main()
{
  // Construction
  gazebo::math::Angle gzAngle(0.1);
  ignition::math::Angle ignAngle(0.1);

  // Get
  std::cout << gzAngle.Radian() << std::endl;
  std::cout << ignAngle.Radian() << std::endl;

  // Output
  std::cout << gzAngle << std::endl;
  std::cout << ignAngle << std::endl;

  // Convert from gazebo to ignition
  ignAngle = gzAngle.Ign();

  // Set gazebo from ignition
  gzAngle = ignAngle;

  return 0;
}
