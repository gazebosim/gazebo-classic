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
/* Desc: Middleman between OGRE and Gazebo
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>

#include <stdint.h>
#include <sys/types.h>
#include <dirent.h>
#include <iostream>
#include <string.h>

#include "rendering/ogre.h"

#include "gazebo_config.h"

#include "transport/Transport.hh"
#include "transport/Node.hh"
#include "transport/Subscriber.hh"
#include "common/Color.hh"
#include "common/Events.hh"
#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/SystemPaths.hh"

#include "rendering/RenderEvents.hh"
#include "rendering/RTShaderSystem.hh"
#include "rendering/WindowManager.hh"
#include "rendering/Scene.hh"
#include "rendering/Grid.hh"
#include "rendering/Visual.hh"
#include "rendering/UserCamera.hh"
#include "rendering/RenderEngine.hh"

using namespace gazebo;
using namespace rendering;


//////////////////////////////////////////////////
/// Constructor
RenderEngine::RenderEngine()
{
  this->logManager = NULL;
  this->root = NULL;

  this->dummyDisplay = NULL;

  this->initialized = false;

  this->connections.push_back( event::Events::ConnectPreRender( 
        boost::bind(&RenderEngine::PreRender, this) ) );
  this->connections.push_back( event::Events::ConnectRender( 
        boost::bind(&RenderEngine::Render, this) ) );
  this->connections.push_back( event::Events::ConnectPostRender( 
        boost::bind(&RenderEngine::PostRender, this) ) );


}


//////////////////////////////////////////////////
/// Destructor
RenderEngine::~RenderEngine()
{
  //this->Fini();
}

//////////////////////////////////////////////////
/// Load the parameters for Ogre
void RenderEngine::Load()
{
  this->CreateContext();

  // Create a new log manager and prevent output from going to stdout
  this->logManager = new Ogre::LogManager();

  std::string logPath = common::SystemPaths::Instance()->GetLogPath();
  logPath += "/ogre.log";

  this->logManager->createLog(logPath, true, false, false);

  if (this->root)
  {
    gzerr << "Attempting to load, but root already exist\n";
    return;
  }

  // Make the root
  try
  {
    this->root = new Ogre::Root();
  }
  catch (Ogre::Exception e)
  {
    gzthrow("Unable to create an Ogre rendering environment, no Root ");
  }

  // Load all the plugins
  this->LoadPlugins();


  // Setup the rendering system, and create the context
  this->SetupRenderSystem();

  // Initialize the root node, and don't create a window
  this->root->initialise(false);

  // Setup the available resources
  this->SetupResources();

  std::stringstream stream;
  stream << (int32_t)this->dummyWindowId;

  WindowManager::Instance()->CreateWindow( stream.str(), 1, 1 );
}

//////////////////////////////////////////////////
// Create a scene
ScenePtr RenderEngine::CreateScene(const std::string &_name, 
                                   bool _enableVisualizations)
{
  ScenePtr scene(new Scene(_name, _enableVisualizations));
  this->scenes.push_back(scene);

  scene->Load();
  if (this->initialized)
    scene->Init();
  else
    gzerr << "RenderEngine is not initialized\n";

  rendering::Events::createScene(_name);

  return scene;
}

// Remove a scene
void RenderEngine::RemoveScene(const std::string &_name)
{
  std::vector<ScenePtr>::iterator iter;

  for (iter = this->scenes.begin(); iter != this->scenes.end(); iter++)
    if ((*iter)->GetName() == _name)
      break;

  if (iter != this->scenes.end())
  {
    RTShaderSystem::Instance()->Clear();
    rendering::Events::removeScene(_name);

    (*iter)->Clear();
    (*iter).reset();
    this->scenes.erase(iter);
  }
}

//////////////////////////////////////////////////
/// Get a scene 
ScenePtr RenderEngine::GetScene(const std::string &_name)
{
  std::vector<ScenePtr>::iterator iter;

  for (iter = this->scenes.begin(); iter != this->scenes.end(); iter++)
    if ((*iter)->GetName() == _name)
      return (*iter);

  return ScenePtr();
}

//////////////////////////////////////////////////
/// Get a scene 
ScenePtr RenderEngine::GetScene(unsigned int index)
{
  if (index < this->scenes.size())
    return this->scenes[index];
  else
  {
    gzerr << "Invalid Scene Index[" << index << "]\n";
    return ScenePtr();
  }
}

//////////////////////////////////////////////////
/// Get the number of scene 
unsigned int RenderEngine::GetSceneCount() const
{
  return this->scenes.size();
}

void RenderEngine::PreRender()
{
  /*if (this->removeScene)
  {
    std::cout << "REnderingEngin::RemoveScene\n";
    this->RemoveScene(this->removeSceneName);
    this->removeScene = false;
    transport::pause_incoming(false);
  }
  else if (this->createScene)
  {
    this->CreateScene(this->createSceneName,true);
    this->createScene = false;
  }
  */

  this->root->_fireFrameStarted();
}

void RenderEngine::Render()
{
}

void RenderEngine::PostRender()
{
  //_fireFrameRenderingQueued needs to be here for CEGUI to work
  this->root->_fireFrameRenderingQueued();
  this->root->_fireFrameEnded();
}

//////////////////////////////////////////////////
// Initialize ogre
void RenderEngine::Init()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->initialized = false;

  Ogre::ColourValue ambient;

  /// Create a dummy rendering context.
  /// This will allow gazebo to run headless. And it also allows OGRE to 
  /// initialize properly

  // Set default mipmap level (NB some APIs ignore this)
  Ogre::TextureManager::getSingleton().setDefaultNumMipmaps( 5 );
  
  // init the resources
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(
      Ogre::TFO_ANISOTROPIC);

  RTShaderSystem::Instance()->Init();

  for (unsigned int i=0; i < this->scenes.size(); i++)
    this->scenes[i]->Init();

  this->initialized = true;
}


//////////////////////////////////////////////////
/// Finalize
void RenderEngine::Fini()
{
  if (!this->initialized)
    return;

  this->node->Fini();
  this->connections.clear();

  // TODO: this was causing a segfault on shutdown
  // Close all the windows first;
  WindowManager::Instance()->Fini();

  RTShaderSystem::Instance()->Fini();

  this->scenes.clear();

  // TODO: this was causing a segfault. Need to debug, and put back in
  if (this->root)
  {
    this->root->shutdown();
    /*const Ogre::Root::PluginInstanceList ll = this->root->getInstalledPlugins();

    for (Ogre::Root::PluginInstanceList::const_iterator iter = ll.begin(); 
         iter != ll.end(); iter++)
    {
      this->root->unloadPlugin((*iter)->getName());
      this->root->uninstallPlugin(*iter);
    }
    */

    try
    {
      delete this->root;
    }
    catch (...) {}
  }
  this->root = NULL;

  delete this->logManager;
  this->logManager = NULL;

  if (this->dummyDisplay)
  {
    glXDestroyContext((Display*)this->dummyDisplay, 
                      (GLXContext)this->dummyContext);
    XDestroyWindow((Display*)this->dummyDisplay, this->dummyWindowId);
    XCloseDisplay((Display*)this->dummyDisplay);
    this->dummyDisplay = NULL;
  }

  this->initialized = false;
}
 
//////////////////////////////////////////////////
// Save
void RenderEngine::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << "<rendering:ogre>\n";
  //this->scenes[0]->Save(prefix,stream);
  stream << prefix << "</rendering:ogre>\n";
}

//////////////////////////////////////////////////
// Load plugins
void RenderEngine::LoadPlugins()
{
  std::list<std::string>::iterator iter;
  std::list<std::string> ogrePaths =
    common::SystemPaths::Instance()->GetOgrePaths();
 
  for (iter=ogrePaths.begin(); 
       iter!=ogrePaths.end(); ++iter)
  {
    std::string path(*iter);
    DIR *dir=opendir(path.c_str()); 

    if (dir == NULL)
    {
      continue;
    }
    closedir(dir);

    std::vector<std::string> plugins;
    std::vector<std::string>::iterator piter;

    plugins.push_back(path+"/RenderSystem_GL.so");
    plugins.push_back(path+"/Plugin_ParticleFX.so");
    plugins.push_back(path+"/Plugin_BSPSceneManager.so");
    plugins.push_back(path+"/Plugin_OctreeSceneManager.so");
    plugins.push_back(path+"/Plugin_CgProgramManager.so");

    for (piter=plugins.begin(); piter!=plugins.end(); piter++)
    {
      try
      {
        // Load the plugin into OGRE
        this->root->loadPlugin(*piter);
      }
      catch (Ogre::Exception e)
      {
        std::string description("Unable to load Ogre Plugin[");
        description.append(*piter);
        description.append("]...Skipping.");
        gzerr << description << "\n";
      }
    }
  }

}

void RenderEngine::AddResourcePath(const std::string &_path)
{
  try
  {
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
        _path, "FileSystem", "General");
  }
  catch (Ogre::Exception)
  {
    gzthrow("Unable to load Ogre Resources.\nMake sure the resources path in the world file is set correctly.");
  }
}

//////////////////////////////////////////////////
// Setup resources
void RenderEngine::SetupResources()
{
  std::vector< std::pair<std::string,std::string> > archNames;
  std::vector< std::pair<std::string, std::string> >::iterator aiter;
  std::list<std::string>::const_iterator iter;
  std::list<std::string> paths = common::SystemPaths::Instance()->GetGazeboPaths();

  for (iter= paths.begin(); iter != paths.end(); iter++)
  {
    DIR *dir;
    if ((dir=opendir((*iter).c_str())) == NULL)
    {
      continue;
    }
    closedir(dir);

    archNames.push_back(std::make_pair((*iter)+"/", "General"));
    archNames.push_back(std::make_pair((*iter)+"/Media", "General"));
    archNames.push_back(std::make_pair((*iter)+"/Media/rtshaderlib", "General") );
    archNames.push_back(std::make_pair((*iter)+"/Media/materials/programs", "General"));
    archNames.push_back(std::make_pair((*iter)+"/Media/materials/scripts", "General"));
    archNames.push_back(std::make_pair((*iter)+"/Media/materials/textures", "General"));
    archNames.push_back(std::make_pair((*iter)+"/Media/models", "General"));
    archNames.push_back(std::make_pair((*iter)+"/Media/fonts", "Fonts"));
    archNames.push_back(std::make_pair((*iter)+"/Media/gui/looknfeel", "LookNFeel"));
    archNames.push_back(std::make_pair((*iter)+"/Media/gui/schemes", "Schemes"));
    archNames.push_back(std::make_pair((*iter)+"/Media/gui/imagesets", "Imagesets"));
    archNames.push_back(std::make_pair((*iter)+"/Media/gui/fonts", "Fonts"));
    archNames.push_back(std::make_pair((*iter)+"/Media/gui/layouts", "Layouts"));
    archNames.push_back(std::make_pair((*iter)+"/Media/gui/animations", "Animations"));

    for (aiter=archNames.begin(); aiter!=archNames.end(); aiter++)
    {
      try
      {
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation( 
            aiter->first, "FileSystem", aiter->second);
      }
      catch (Ogre::Exception)
      {
        gzthrow("Unable to load Ogre Resources.\nMake sure the resources path in the world file is set correctly.");
      }
    }
  }

}

//////////////////////////////////////////////////
// Setup render system
void RenderEngine::SetupRenderSystem()
{
  Ogre::RenderSystem *renderSys;
  const Ogre::RenderSystemList *rsList;

  // Set parameters of render system (window size, etc.)
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
    rsList = this->root->getAvailableRenderers();
#else
    rsList = &(this->root->getAvailableRenderers());
#endif

  int c = 0;

  renderSys = NULL;

  do
  {
    if (c == (int)rsList->size())
      break;

    renderSys = rsList->at(c);
    c++;
  }
  while (renderSys->getName().compare("OpenGL Rendering Subsystem")!= 0);

  if (renderSys == NULL)
    gzthrow( "unable to find rendering system" );

  // We operate in windowed mode
  renderSys->setConfigOption("Full Screen","No");

  /// We used to allow the user to set the RTT mode to PBuffer, FBO, or Copy. 
  ///   Copy is slow, and there doesn't seem to be a good reason to use it
  ///   PBuffer limits the size of the renderable area of the RTT to the
  ///           size of the first window created.
  ///   FBO seem to be the only good option
  renderSys->setConfigOption("RTT Preferred Mode", "FBO");

  renderSys->setConfigOption("FSAA", "4");

  this->root->setRenderSystem(renderSys);
}

//////////////////////////////////////////////////
// Returns true if the graphics card support GLSL
bool RenderEngine::HasGLSL()
{
  const Ogre::RenderSystemCapabilities *capabilities;
  Ogre::RenderSystemCapabilities::ShaderProfiles profiles;
  Ogre::RenderSystemCapabilities::ShaderProfiles::const_iterator iter;

  capabilities = this->root->getRenderSystem()->getCapabilities();
  profiles = capabilities->getSupportedShaderProfiles();

  iter = std::find(profiles.begin(), profiles.end(), "glsl");

  return iter != profiles.end();
}


void RenderEngine::CreateContext()
{
  this->dummyDisplay = XOpenDisplay(0);
  if (!this->dummyDisplay) 
    gzthrow(std::string("Can't open display: ") + XDisplayName(0) + "\n");

  int screen = DefaultScreen(this->dummyDisplay);

  int attribList[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, 16, 
    GLX_STENCIL_SIZE, 8, None };

  XVisualInfo *dummyVisual = glXChooseVisual((Display*)this->dummyDisplay, 
      screen, (int *)attribList);

  this->dummyWindowId = XCreateSimpleWindow((Display*)this->dummyDisplay, 
      RootWindow((Display*)this->dummyDisplay, screen), 0, 0, 1, 1, 0, 0, 0);

  this->dummyContext = glXCreateContext((Display*)this->dummyDisplay, 
      dummyVisual, NULL, 1);

  glXMakeCurrent((Display*)this->dummyDisplay, 
      this->dummyWindowId, (GLXContext)this->dummyContext);
}
