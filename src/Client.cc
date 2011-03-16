#include "common/XMLConfig.hh"
#include "common/GazeboError.hh"
#include "common/GazeboConfig.hh"

#include "gui/SimulationApp.hh"
#include "rendering/RenderEngine.hh"
#include "rendering/RenderState.hh"
#include "rendering/Scene.hh"

#include "transport/IOManager.hh"

#include "Client.hh"

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


Client::Client()
  : renderEngineEnabled(true), guiEnabled(true)
{
  this->gui = NULL;

  // load the configuration options 
  try
  {
    common::GazeboConfig::Instance()->Load();
  }
  catch (common::GazeboError e)
  {
    gzthrow("Error loading the Gazebo configuration file, check the .gazeborc file on your HOME directory \n" << e); 
  }
}

Client::~Client()
{
  if (this->gui)
    delete this->gui;
}

void Client::Load(const std::string &filename)
{
  // Load the world file
  gazebo::common::XMLConfig *xmlFile = new gazebo::common::XMLConfig();

  try
  {
    if (!filename.empty())
      xmlFile->Load(filename);
    else
      xmlFile->LoadString(default_config);
  }
  catch (common::GazeboError e)
  {
    gzthrow("The XML config file can not be loaded, please make sure is a correct file\n" << e); 
  }

  common::XMLConfigNode *rootNode(xmlFile->GetRootNode());
  if (!rootNode || rootNode->GetName() != "gazebo")
    gzthrow("Invalid xml. Needs a root node with the <gazebo> tag");


  common::XMLConfigNode *configNode = rootNode->GetChild("config");
  if (!configNode)
    gzthrow("Invalid xml. Needs a <config> tag");
  
  // Load the Ogre rendering system
  rendering::RenderEngine::Instance()->Load(configNode);

  
  // Create and initialize the Gui
  if (this->renderEngineEnabled && this->guiEnabled)
  {
    try
    {
      common::XMLConfigNode *childNode = NULL;
      if (rootNode)
        childNode = configNode->GetChild("gui");

      int width=0;
      int height=0;
      int x = 0;
      int y = 0;

      if (childNode)
      {
        width = childNode->GetTupleInt("size", 0, 800);
        height = childNode->GetTupleInt("size", 1, 600);
        x = childNode->GetTupleInt("pos",0,0);
        y = childNode->GetTupleInt("pos",1,0);
      }

      // Create the GUI
      if (!this->gui && (childNode || !rootNode))
      {
        this->gui = new gui::SimulationApp();
        this->gui->Load();
      }
    }
    catch (common::GazeboError e)
    {
      gzthrow( "Error loading the GUI\n" << e);
    }
  }
  else
  {
    this->gui = NULL;
  }

  //Initialize RenderEngine
  if (this->renderEngineEnabled)
  {
    try
    {
      rendering::RenderEngine::Instance()->Init(configNode);
    }
    catch (common::GazeboError e)
    {
      gzthrow("Failed to Initialize the Rendering engine subsystem\n" << e );
    }
  }

  // Initialize the GUI
  if (this->gui)
  {
    this->gui->Init();
  }

  delete xmlFile;
}

void Client::Run()
{
  rendering::Scene *scene = new rendering::Scene("default");
  scene->Load(NULL);
  scene->Init();

  this->gui->ViewScene(scene);

  this->gui->Run();
}
