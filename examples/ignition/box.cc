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

#include <gazebo/math/Box.hh>
#include <ignition/math/Box.hh>

/////////////////////////////////////////////////
// Migration from gazebo::math::Box to ignition::math::Box examples
// See Ignition documentation http://ignitionrobotics.org/libraries/math
int main()
{
  // Construction
  gazebo::math::Box gzBox(gazebo::math::Vector3(1, 2, 3),
                          gazebo::math::Vector3(4, 5, 6));
  ignition::math::Box ignBox(1, 2, 3, 4, 5, 6);

  // Get
  std::cout << gzBox.min << std::endl;
  std::cout << ignBox.Min() << std::endl;

  // Output
  std::cout << gzBox << std::endl;
  std::cout << ignBox << std::endl;

  // Convert from gazebo to ignition
  ignBox = gzBox.Ign();

  // Set gazebo from ignition
  gzBox = ignBox;

  return 0;
}
