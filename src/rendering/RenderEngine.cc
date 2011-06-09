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

#include "common/Color.hh"
#include "common/Events.hh"
#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/GazeboConfig.hh"
#include "common/Global.hh"
#include "common/XMLConfig.hh"

//#include "rendering/RTShaderSystem.hh"
#include "rendering/WindowManager.hh"
#include "rendering/Scene.hh"
#include "rendering/Grid.hh"
#include "rendering/Visual.hh"
#include "rendering/UserCamera.hh"
#include "rendering/MovableText.hh"
#include "rendering/RenderEngine.hh"

using namespace gazebo;
using namespace rendering;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
RenderEngine::RenderEngine()
{
  this->headless = false;

  this->logManager = NULL;
  this->root = NULL;

  this->dummyDisplay = false;

  this->initialized = false;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
RenderEngine::~RenderEngine()
{
  this->Fini();
  delete this->logManager;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the parameters for Ogre
void RenderEngine::Load(common::XMLConfigNode *rootNode)
{
  // Create a new log manager and prevent output from going to stdout
  this->logManager = new Ogre::LogManager();
  this->logManager->createLog("Ogre.log", true, false, false);
  

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
}

////////////////////////////////////////////////////////////////////////////////
// Create a scene
ScenePtr RenderEngine::CreateScene(const std::string &name)
{
  ScenePtr scene( new Scene(name) );
  scene->Load(NULL);

  if (this->initialized)
    scene->Init();

  scene->SetType(Scene::GENERIC);
  scene->SetAmbientColor(common::Color(0.5, 0.5, 0.5));
  scene->SetBackgroundColor(common::Color(0.5, 0.5, 0.5, 1.0));
  scene->CreateGrid( 10, 1, 0.03, common::Color(1,1,1,1));

  this->scenes.push_back(scene);
  return scene;
}

////////////////////////////////////////////////////////////////////////////////
// Remove a scene
void RenderEngine::RemoveScene(const std::string &name)
{
  std::vector<ScenePtr>::iterator iter;

  for (iter = this->scenes.begin(); iter != this->scenes.end(); iter++)
    if ((*iter)->GetName() == name)
      break;

  if (iter != this->scenes.end())
  {
    this->scenes.erase(iter);
  }
}

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// Get the number of scene 
unsigned int RenderEngine::GetSceneCount() const
{
  return this->scenes.size();
}


////////////////////////////////////////////////////////////////////////////////
/// Update all the scenes 
void RenderEngine::UpdateScenes()
{
  event::Events::preRenderSignal();

  this->root->_fireFrameStarted();

  event::Events::renderSignal();

  this->root->_fireFrameRenderingQueued();

  this->root->_fireFrameEnded();

  event::Events::postRenderSignal();
}

////////////////////////////////////////////////////////////////////////////////
// Initialize ogre
void RenderEngine::Init()
{
  this->initialized = false;

  Ogre::ColourValue ambient;

  /// Create a dummy rendering context.
  /// This will allow gazebo to run headless. And it also allows OGRE to 
  /// initialize properly
  //if (this->headless)
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

    std::stringstream stream;
    stream << (int32_t)this->dummyWindowId;

    WindowManager::Instance()->CreateWindow( stream.str(), 1, 1 );
  }

  // Set default mipmap level (NB some APIs ignore this)
  Ogre::TextureManager::getSingleton().setDefaultNumMipmaps( 5 );
  
  // init the resources
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(Ogre::TFO_ANISOTROPIC);

  /*
  if (this->HasGLSL())
    RTShaderSystem::Instance()->Init();

  if (this->HasGLSL())
    RTShaderSystem::Instance()->UpdateShaders();
    */

  for (unsigned int i=0; i < this->scenes.size(); i++)
    this->scenes[i]->Init();

  this->initialized = true;
}


////////////////////////////////////////////////////////////////////////////////
/// Finalize
void RenderEngine::Fini()
{
  // Close all the windows first;
  WindowManager::Instance()->Fini();

  this->scenes.clear();

  // TODO: this was causing a segfault. Need to debug, and put back in
 /*if (this->root)
  {
    const Ogre::Root::PluginInstanceList ll = this->root->getInstalledPlugins();

    for (Ogre::Root::PluginInstanceList::const_iterator iter = ll.begin(); iter != ll.end(); iter++)
    {
      this->root->unloadPlugin((*iter)->getName());
      this->root->uninstallPlugin(*iter);
    }
    delete this->root;
  }
  this->root = NULL;

  if (this->logManager)
  {
    this->logManager->destroyLog("Ogre.log");
    delete this->logManager;
  }
  this->logManager = NULL;
  */


  if (this->dummyDisplay)
  {
    glXDestroyContext((Display*)this->dummyDisplay, 
                      (GLXContext)this->dummyContext);
    XDestroyWindow((Display*)this->dummyDisplay, this->dummyWindowId);
    XCloseDisplay((Display*)this->dummyDisplay);
  }
  this->dummyDisplay = false;
  this->initialized = false;
}
 
////////////////////////////////////////////////////////////////////////////////
// Save
void RenderEngine::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << "<rendering:ogre>\n";
  //this->scenes[0]->Save(prefix,stream);
  stream << prefix << "</rendering:ogre>\n";
}

////////////////////////////////////////////////////////////////////////////////
// Load plugins
void RenderEngine::LoadPlugins()
{
  std::list<std::string>::iterator iter;
  std::list<std::string> ogrePaths = common::GazeboConfig::Instance()->GetOgrePaths();
 
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
    //plugins.push_back(path+"/Plugin_CgProgramManager.so");

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

////////////////////////////////////////////////////////////////////////////////
// Setup resources
void RenderEngine::SetupResources()
{
  std::vector<std::string> archNames;
  std::vector<std::string>::iterator aiter;
  std::list<std::string>::iterator iter;

  for (iter= common::GazeboConfig::Instance()->GetGazeboPaths().begin();
       iter!=common::GazeboConfig::Instance()->GetGazeboPaths().end(); iter++)
  {
    DIR *dir;
    if ((dir=opendir((*iter).c_str())) == NULL)
    {
      continue;
    }
    closedir(dir);

    archNames.push_back((*iter)+"/");
    archNames.push_back((*iter)+"/Media");
    archNames.push_back((*iter)+"/Media/fonts");
    archNames.push_back((*iter)+"/Media/rtshaderlib");
    archNames.push_back((*iter)+"/Media/materials/programs");
    archNames.push_back((*iter)+"/Media/materials/scripts");
    archNames.push_back((*iter)+"/Media/materials/textures");
    archNames.push_back((*iter)+"/Media/models");
    archNames.push_back((*iter)+"/Media/sets");
    archNames.push_back((*iter)+"/Media/maps");

    //we want to add all the material files of the sets
    if ((dir=opendir(((*iter)+"/Media/sets").c_str()))!= NULL)
    {
      std::string filename;
      struct dirent *dir_entry_p;
      while ( (dir_entry_p = readdir(dir))!=NULL )
      {
        filename =(*iter)+"/Media/sets/"+ dir_entry_p->d_name;
        archNames.push_back(filename);
      }
      closedir(dir);
    }

    for (aiter=archNames.begin(); aiter!=archNames.end(); aiter++)
    {
      try
      {
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation( *aiter, "FileSystem", "General");
      }
      catch (Ogre::Exception)
      {
        gzthrow("Unable to load Ogre Resources.\nMake sure the resources path in the world file is set correctly.");
      }
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
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

  renderSys->setConfigOption("FSAA", "2");

  this->root->setRenderSystem(renderSys);
}

////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
// True if the gui is to be used
void RenderEngine::SetHeadless( bool enabled )
{
  this->headless = enabled;
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if the gui is enabled
bool RenderEngine::GetHeadless() const
{
  return this->headless;
}
