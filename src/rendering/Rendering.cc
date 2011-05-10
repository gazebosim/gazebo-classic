#include "common/XMLConfig.hh"
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


bool rendering::load(const std::string &filename)
{
  bool result = true;

  // Load the world file
  common::XMLConfig *xmlFile = new common::XMLConfig();

  try
  {
    if (!filename.empty())
      xmlFile->Load(filename);
    else
      xmlFile->LoadString(default_config);
  }
  catch (common::Exception e)
  {
    gzthrow("The XML config file can not be loaded, please make sure is a correct file\n" << e); 
  }

  // Get the root node, and make sure we have a gazebo config
  common::XMLConfigNode *rootNode(xmlFile->GetRootNode());
  if (!rootNode || rootNode->GetName() != "gazebo")
    gzthrow("Invalid xml. Needs a root node with the <gazebo> tag");

  // Get the config node
  common::XMLConfigNode *configNode = rootNode->GetChild("config");
  if (!configNode)
    gzthrow("Invalid xml. Needs a <config> tag");

  try
  {
    rendering::RenderEngine::Instance()->Load(configNode);
  }
  catch(common::Exception e)
  {
    result = false;
    gzerr << "Failed to load the Rendering engine subsystem\n" << e ;
  }

  delete xmlFile;

  return result;
}

bool rendering::init()
{
  bool result = true;

  //rendering::RenderEngine::Instance()->SetHeadless(create_dummy_window);

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

