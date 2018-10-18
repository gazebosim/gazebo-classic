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

#include <gazebo/math/Vector3.hh>
#include <ignition/math/Vector3.hh>

/////////////////////////////////////////////////
// Migration from gazebo::math::Vector3 to ignition::math::Vector3 examples
// See Ignition documentation http://ignitionrobotics.org/libraries/math
int main()
{
  // Construction
  gazebo::math::Vector3 gzVec(1, 2, 3);
  ignition::math::Vector3d ignVec(1, 2, 3);

  // Set
  gzVec.x = 2.5;
  ignVec.X() = 3.5;
  ignVec.X(4.5);

  // Get
  std::cout << gzVec.x << std::endl;
  std::cout << ignVec.X() << std::endl;

  // Output
  std::cout << gzVec << std::endl;
  std::cout << ignVec << std::endl;

  // Convert from ignition to gazebo
  gzVec = ignVec;

  // Convert gazebo to igntion
  ignVec = gzVec.Ign();

  return 0;
}
