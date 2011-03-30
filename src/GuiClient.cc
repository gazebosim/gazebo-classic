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
#include "common/XMLConfig.hh"
#include "common/GazeboError.hh"
#include "common/GazeboConfig.hh"

#include "gui/SimulationApp.hh"

#include "rendering/RenderEngine.hh"
#include "rendering/RenderState.hh"
#include "rendering/Scene.hh"

#include "transport/Transport.hh"

#include "GuiClient.hh"

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


GuiClient::GuiClient()
  : renderEngineEnabled(true), guiEnabled(true)
{
  this->quit = false;
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

  // Start the transport system by connecting to the master.
  transport::init();
}

GuiClient::~GuiClient()
{
  if (this->gui)
    delete this->gui;
}

void GuiClient::Load(const std::string &filename)
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

void GuiClient::Run()
{
  rendering::Scene *scene = new rendering::Scene("default");
  scene->Load(NULL);
  scene->Init();

  transport::PublisherPtr pub = transport::advertise<msgs::Request>("/gazebo/default/publish_scene");
  msgs::Request req;
  req.set_request("publish");
  pub->Publish(req);

  this->gui->ViewScene(scene);

  this->gui->Run();
}

void GuiClient::Quit()
{
  this->quit = true;
}

