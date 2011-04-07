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

#include "transport/Transport.hh"

#include "common/XMLConfig.hh"
#include "common/Events.hh"

#include "gui/SimulationFrame.hh"
#include "gui/SimulationApp.hh"

#include "rendering/Rendering.hh"

using namespace gazebo;
using namespace gui;

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

////////////////////////////////////////////////////////////////////////////////
// Constructor
SimulationApp::SimulationApp()
  : frame (NULL)
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor 
SimulationApp::~SimulationApp()
{
  wxTheApp->OnExit();
}

////////////////////////////////////////////////////////////////////////////////
/// Load the simulation app
void SimulationApp::Load(const std::string &filename)
{
  // Load the world file
  common::XMLConfig *xmlFile = new common::XMLConfig();

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

  // Get the root node, and make sure we have a gazebo config
  common::XMLConfigNode *rootNode(xmlFile->GetRootNode());
  if (!rootNode || rootNode->GetName() != "gazebo")
    gzthrow("Invalid xml. Needs a root node with the <gazebo> tag");

  // Get the config node
  common::XMLConfigNode *configNode = rootNode->GetChild("config");
  if (!configNode)
    gzthrow("Invalid xml. Needs a <config> tag");
 
  // Get the gui node
  common::XMLConfigNode *guiNode = configNode->GetChild("gui");
  if (!guiNode)
    gzthrow("Invalid xml. Needs a <gui> tag");

  // Load the Ogre rendering system
  if (!rendering::load(configNode))
    gzthrow("Failed to load the rendering engine");

  delete xmlFile;
}

////////////////////////////////////////////////////////////////////////////////
// Run the gui
void SimulationApp::Run()
{
  wxTheApp->OnRun();
}

////////////////////////////////////////////////////////////////////////////////
// Init the simulation app
void SimulationApp::Init()
{
  int local_argc = 0;
  char **local_argv = new char*[local_argc];
  //for (int i=0; i < local_argc; i++)
    //local_argv[i] = strdup(wxString(argv[i]).mb_str());

  wxEntryStart(local_argc, local_argv);
  wxTheApp->OnInit();

}

////////////////////////////////////////////////////////////////////////////////
/// On Init
bool SimulationApp::OnInit()
{
  std::cout << "On Init\n";
  this->frame = new SimulationFrame(NULL);
  this->frame->Show();
  this->SetTopWindow(this->frame);

  this->frame->Init();

  if (!rendering::init(false))
    gzthrow("Failed to initialized the rendering engine\n");

  rendering::ScenePtr scene = rendering::create_scene("default");
  this->ViewScene(scene);

  // Send a request to get the current world state
  // TODO: Use RPC or some service call to get this properly
  transport::PublisherPtr pub = transport::advertise<msgs::Request>("/gazebo/default/publish_scene");
  msgs::Request req;
  req.set_request("publish");
  pub->Publish(req);


  //Connect( wxID_ANY, wxEVT_IDLE, wxIdleEventHandler(SimulationApp::OnIdle) );
  Connect( this->timer.GetId(), wxEVT_TIMER, wxTimerEventHandler(SimulationApp::OnIdle), NULL, this );
  this->timer.Start(33);
  std::cout << "On Init done\n";

  return true;
}

void SimulationApp::OnIdle(wxTimerEvent &evt)
{
  this->frame->Update();

  event::Events::preRenderSignal();

  // Tell all the cameras to render
  event::Events::renderSignal();

  event::Events::postRenderSignal();
}


////////////////////////////////////////////////////////////////////////////////
/// Save the gui params in xml format
void SimulationApp::Save(std::string &prefix, std::ostream &stream)
{
}

////////////////////////////////////////////////////////////////////////////////
/// View a specific world
void SimulationApp::ViewScene( rendering::ScenePtr scene )
{
  this->frame->ViewScene(scene);
}
