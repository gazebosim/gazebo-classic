
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

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/FPSViewController.hh>
#include <gazebo/rendering/UserCamera.hh>

int main(int _argc, char **_argv)
{
  // Initialize gazebo.
  gazebo::setupServer(_argc, _argv);

  gazebo::gui::get_active_camera()->SetViewController(gazebo::rendering::FPSViewController::GetTypeString());
  // Load a world
  gazebo::physics::WorldPtr world = gazebo::loadWorld("worlds/empty.world");


  while (true)
  {
    // Run simulation for 100 steps.
    gazebo::runWorld(world, 100);
  }

  // Close everything.
  gazebo::shutdown();
}
