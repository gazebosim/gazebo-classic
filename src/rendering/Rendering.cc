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
#include <boost/thread.hpp>
#include "common/Exception.hh"
#include "common/Console.hh"

#include "rendering/RenderEngine.hh"
#include "Rendering.hh"

using namespace gazebo;

const std::string default_config =
"<?xml version='1.0'?>\
<gazebo>\
  <config>\
    <verbosity>4</verbosity>\
    <gui>\
      <size>800 600</size>\
      <pos>0 0</pos>\
    </gui>\
  </config>\
</gazebo>\
";


bool rendering::load()
{
  bool result = true;

  try
  {
    rendering::RenderEngine::Instance()->Load();
  }
  catch(common::Exception e)
  {
    result = false;
    gzerr << "Failed to load the Rendering engine subsystem\n" << e ;
  }

  return result;
}

bool rendering::init()
{
  bool result = true;

  //Initialize RenderEngine
  try
  {
    rendering::RenderEngine::Instance()->Init();
  }
  catch (common::Exception e)
  {
    result = false;
    gzerr <<"Failed to Initialize the Rendering engine subsystem\n" << e ;
  }

  return result;
}

bool rendering::fini()
{
  rendering::RenderEngine::Instance()->Fini();
  return true;
}

rendering::ScenePtr rendering::create_scene(const std::string &name)
{
  ScenePtr scene;

  // Create a default scene for the gui
  try
  {
    scene = rendering::RenderEngine::Instance()->CreateScene(name);
  }
  catch (common::Exception e)
  {
    gzerr <<"Failed to create a scene in the Rendering engine" << e ;
  }
  

  return scene;
}

