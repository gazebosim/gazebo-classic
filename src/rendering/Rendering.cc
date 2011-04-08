#include "common/XMLConfig.hh"
#include "common/Exception.hh"
#include "common/Console.hh"

#include "rendering/RenderEngine.hh"
#include "Rendering.hh"

using namespace gazebo;

bool rendering::load(common::XMLConfigNode *node)
{
  bool result = true;

  try
  {
    rendering::RenderEngine::Instance()->Load(node);
  }
  catch(common::Exception e)
  {
    result = false;
    gzerr << "Failed to load the Rendering engine subsystem\n" << e ;
  }

  return result;
}

bool rendering::init(bool create_dummy_window)
{
  bool result = true;

  rendering::RenderEngine::Instance()->SetHeadless(create_dummy_window);

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
