/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <string>
#include <iostream>
#include <functional>
#include <boost/filesystem.hpp>
#include <sys/types.h>

// Not Apple or Windows
#if not defined(__APPLE__) && not defined(_WIN32)
# include <X11/Xlib.h>
# include <X11/Xutil.h>
# include <GL/glx.h>
#endif

#ifndef _WIN32
  #include <dirent.h>
#else
  #include "gazebo/common/win_dirent.h"
#endif

#include "gazebo/gazebo_config.h"

#include <ignition/common/Profiler.hh>

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/SystemPaths.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Material.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/WindowManager.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/RenderEnginePrivate.hh"

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
RenderEngine::RenderEngine()
  : dataPtr(new RenderEnginePrivate)
{
  this->dataPtr->logManager = NULL;
  this->dataPtr->root = NULL;

#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
  this->dataPtr->overlaySystem = NULL;
#endif

  this->dummyDisplay = NULL;

  this->dataPtr->initialized = false;

  this->dataPtr->renderPathType = NONE;
  this->dataPtr->windowManager.reset(new WindowManager);
}

//////////////////////////////////////////////////
RenderEngine::~RenderEngine()
{
  // this->Fini();
}

//////////////////////////////////////////////////
void RenderEngine::Load()
{
  if (!this->CreateContext())
  {
    gzwarn << "Unable to create X window. Rendering will be disabled\n";
    return;
  }

  if (!this->dataPtr->root)
  {
    this->dataPtr->connections.push_back(event::Events::ConnectPreRender(
          std::bind(&RenderEngine::PreRender, this)));
    this->dataPtr->connections.push_back(event::Events::ConnectRender(
          std::bind(&RenderEngine::Render, this)));
    this->dataPtr->connections.push_back(event::Events::ConnectPostRender(
          std::bind(&RenderEngine::PostRender, this)));

    // Create a new log manager and prevent output from going to stdout
    this->dataPtr->logManager = new Ogre::LogManager();

    std::string logPath = common::SystemPaths::Instance()->GetLogPath();
    logPath += "/ogre.log";

    this->dataPtr->logManager->createLog(logPath, true, false, false);

    // Make the root
    try
    {
      // empty strings for config filenames (plugins.cfg and ogre.cfg)
      // so ogre doesn't try to look for them.
      this->dataPtr->root = new Ogre::Root("", "");
    }
    catch(Ogre::Exception &e)
    {
      gzthrow("Unable to create an Ogre rendering environment, no Root ");
    }

#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
    // OgreOverlay is a component on its own in ogre 1.9 so must manually
    // initialize it. Must be created after this->dataPtr->root,
    // but before this->dataPtr->root is initialized.
    if (!this->dataPtr->overlaySystem)
      this->dataPtr->overlaySystem = new Ogre::OverlaySystem();
#endif

    // Load all the plugins
    this->LoadPlugins();

    // Setup the rendering system, and create the context
    this->SetupRenderSystem();

    // Initialize the root node, and don't create a window
    this->dataPtr->root->initialise(false);

    // Setup the available resources
    this->SetupResources();
  }

  // Create a 1x1 render window so that we can grab a GL context. Based on
  // testing, this is a hard requirement by Apple. We also need it to
  // properly initialize GLWidget and UserCameras. See the GLWidget
  // constructor.
  this->dataPtr->windowManager->CreateWindow(
      std::to_string(this->dummyWindowId), 1, 1);

  this->CheckSystemCapabilities();
}

//////////////////////////////////////////////////
ScenePtr RenderEngine::CreateScene(const std::string &_name,
                                   bool _enableVisualizations,
                                   bool _isServer)
{
  if (this->dataPtr->renderPathType == NONE)
    return ScenePtr();

  if (!this->dataPtr->initialized)
  {
    gzerr << "RenderEngine is not initialized\n";
    return ScenePtr();
  }

  ScenePtr scene;

  try
  {
    scene.reset(new Scene(_name, _enableVisualizations, _isServer));
  }
  catch(...)
  {
    gzerr << "Failed to instantiate a scene\n";
    scene.reset();
    return scene;
  }

  try
  {
    scene->Load();
  }
  catch(...)
  {
    gzerr << "Failed to load scene\n";
    scene.reset();
    return scene;
  }

  try
  {
    scene->Init();
  }
  catch(...)
  {
    gzerr << "Failed to initialize scene\n";
    scene.reset();
    return scene;
  }

  this->dataPtr->scenes.push_back(scene);

  rendering::Events::createScene(_name);

  return scene;
}

//////////////////////////////////////////////////
void RenderEngine::RemoveScene(const std::string &_name)
{
  if (this->dataPtr->renderPathType == NONE)
    return;

  for (auto iter = this->dataPtr->scenes.begin();
      iter != this->dataPtr->scenes.end(); ++iter)
  {
    if ((*iter)->Name() == _name)
    {
      rendering::Events::removeScene(_name);

      (*iter)->Clear();
      (*iter).reset();
      this->dataPtr->scenes.erase(iter);
      return;
    }
  }
}

//////////////////////////////////////////////////
ScenePtr RenderEngine::GetScene(const std::string &_name)
{
  if (this->dataPtr->renderPathType == NONE)
    return ScenePtr();

  for (const auto &scene : this->dataPtr->scenes)
  {
    if (_name.empty() || scene->Name() == _name)
      return scene;
  }

  return ScenePtr();
}

//////////////////////////////////////////////////
ScenePtr RenderEngine::GetScene(unsigned int index)
{
  if (index < this->dataPtr->scenes.size())
    return this->dataPtr->scenes[index];
  else
  {
    gzerr << "Invalid Scene Index[" << index << "]\n";
    return ScenePtr();
  }
}

//////////////////////////////////////////////////
unsigned int RenderEngine::SceneCount() const
{
  return this->dataPtr->scenes.size();
}

//////////////////////////////////////////////////
void RenderEngine::PreRender()
{
  IGN_PROFILE("rendering::RenderEngine::PreRender");
  this->dataPtr->root->_fireFrameStarted();
}

//////////////////////////////////////////////////
void RenderEngine::Render()
{
  IGN_PROFILE("rendering::RenderEngine::Render");
}

//////////////////////////////////////////////////
void RenderEngine::PostRender()
{
  IGN_PROFILE("rendering::RenderEngine::PostRender");
  // _fireFrameRenderingQueued was here for CEGUI to work. Leaving because
  // it shouldn't harm anything, and we don't want to introduce
  // a regression.
  this->dataPtr->root->_fireFrameRenderingQueued();
  this->dataPtr->root->_fireFrameEnded();
}

//////////////////////////////////////////////////
void RenderEngine::Init()
{
  if (this->dataPtr->renderPathType == NONE)
  {
    gzwarn << "Cannot initialize render engine since "
           << "render path type is NONE. Ignore this warning if "
           << "rendering has been turned off on purpose.\n";
    return;
  }

  this->dataPtr->initialized = false;

  Ogre::ColourValue ambient;

  /// Create a dummy rendering context.
  /// This will allow gazebo to run headless. And it also allows OGRE to
  /// initialize properly

  // Set default mipmap level (NB some APIs ignore this)
  Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

  // init the resources
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(
      Ogre::TFO_ANISOTROPIC);

  RTShaderSystem::Instance()->Init();
  rendering::Material::CreateMaterials();

  for (unsigned int i = 0; i < this->dataPtr->scenes.size(); i++)
    this->dataPtr->scenes[i]->Init();

  this->dataPtr->initialized = true;
}

//////////////////////////////////////////////////
void RenderEngine::Fini()
{
  // TODO: this was causing a segfault on shutdown
  // Windows are created on load so clear them even
  // if render engine is not initialized
  this->dataPtr->windowManager->Fini();

  if (!this->dataPtr->initialized)
    return;

  this->dataPtr->connections.clear();

  RTShaderSystem::Instance()->Fini();

  // Deallocate memory for every scene
  while (!this->dataPtr->scenes.empty())
  {
    this->RemoveScene(this->dataPtr->scenes.front()->Name());
  }

#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
  delete this->dataPtr->overlaySystem;
  this->dataPtr->overlaySystem = NULL;
#endif

  // TODO: this was causing a segfault. Need to debug, and put back in
  if (this->dataPtr->root)
  {
    /*const Ogre::Root::PluginInstanceList ll =
     this->dataPtr->root->getInstalledPlugins();

    for (Ogre::Root::PluginInstanceList::const_iterator iter = ll.begin();
         iter != ll.end(); iter++)
    {
      this->dataPtr->root->unloadPlugin((*iter)->getName());
      this->dataPtr->root->uninstallPlugin(*iter);
    }
    */

    try
    {
      delete this->dataPtr->root;
    }
    catch(...)
    {
    }
  }
  this->dataPtr->root = NULL;

  delete this->dataPtr->logManager;
  this->dataPtr->logManager = NULL;

  for (unsigned int i = 0; i < this->dataPtr->scenes.size(); ++i)
    this->dataPtr->scenes[i].reset();
  this->dataPtr->scenes.clear();

  // Not Apple or Windows
# if not defined(__APPLE__) && not defined(_WIN32)
  if (this->dummyDisplay)
  {
    glXDestroyContext(static_cast<Display*>(this->dummyDisplay),
                      static_cast<GLXContext>(this->dummyContext));
    XDestroyWindow(static_cast<Display*>(this->dummyDisplay),
                   this->dummyWindowId);
    XCloseDisplay(static_cast<Display*>(this->dummyDisplay));
    this->dummyDisplay = NULL;
  }
# endif

  this->dataPtr->initialized = false;
}

//////////////////////////////////////////////////
void RenderEngine::LoadPlugins()
{
  std::list<std::string>::iterator iter;
  std::list<std::string> ogrePaths =
    common::SystemPaths::Instance()->GetOgrePaths();

  for (iter = ogrePaths.begin();
       iter != ogrePaths.end(); ++iter)
  {
    std::string path(*iter);
    DIR *dir = opendir(path.c_str());

    if (dir == NULL)
    {
      continue;
    }
    closedir(dir);

    std::vector<std::string> plugins;
    std::vector<std::string>::iterator piter;

#ifdef __APPLE__
    std::string extension = ".dylib";
#elif defined(_WIN32)
    std::string extension = ".dll";
#else
    std::string extension = ".so";
#endif

    plugins.push_back(path+"/RenderSystem_GL");
    plugins.push_back(path+"/Plugin_ParticleFX");
    plugins.push_back(path+"/Plugin_BSPSceneManager");
    plugins.push_back(path+"/Plugin_OctreeSceneManager");

#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 11
    // OGRE 1.11 moved FreeImage codec support to a plugin
    // See: https://github.com/OGRECave/ogre/blob/master/Docs/1.11-Notes.md
    plugins.push_back(path + "/Codec_FreeImage");
#endif

#ifdef HAVE_OCULUS
    plugins.push_back(path+"/Plugin_CgProgramManager");
#endif

    for (piter = plugins.begin(); piter != plugins.end(); ++piter)
    {
      try
      {
        // Load the plugin into OGRE
        this->dataPtr->root->loadPlugin(*piter+extension);
      }
      catch(Ogre::Exception &e)
      {
        try
        {
          // Load the debug plugin into OGRE
          this->dataPtr->root->loadPlugin(*piter+"_d"+extension);
        }
        catch(Ogre::Exception &ed)
        {
          if ((*piter).find("RenderSystem") != std::string::npos)
          {
            gzerr << "Unable to load Ogre Plugin[" << *piter
                  << "]. Rendering will not be possible. "
                  << "Make sure you have installed OGRE and Gazebo properly.\n";
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void RenderEngine::AddResourcePath(const std::string &_uri)
{
  if (_uri.find("__default__") != std::string::npos || _uri.empty())
    return;

  std::string path = common::find_file_path(_uri);

  if (path.empty())
  {
    gzerr << "URI doesn't exist[" << _uri << "]\n";
    return;
  }

  try
  {
    path = boost::filesystem::path(path).make_preferred().string();
    if (!Ogre::ResourceGroupManager::getSingleton().resourceLocationExists(
          path, "General"))
    {
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          path, "FileSystem", "General", true);
      Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(
          "General");

      // Parse all material files in the path if any exist
      boost::filesystem::path dir(path);

      if (boost::filesystem::exists(dir) &&
          boost::filesystem::is_directory(dir))
      {
        std::vector<boost::filesystem::path> paths;
        std::copy(boost::filesystem::directory_iterator(dir),
            boost::filesystem::directory_iterator(),
            std::back_inserter(paths));
        std::sort(paths.begin(), paths.end());

        // Iterate over all the models in the current gazebo path
        for (std::vector<boost::filesystem::path>::iterator dIter =
            paths.begin(); dIter != paths.end(); ++dIter)
        {
          if (dIter->filename().extension() == ".material")
          {
            boost::filesystem::path fullPath = path / dIter->filename();
            fullPath = fullPath.make_preferred();

            Ogre::DataStreamPtr stream =
              Ogre::ResourceGroupManager::getSingleton().openResource(
                  fullPath.string(), "General");

            // There is a material file under there somewhere, read the thing in
            try
            {
              Ogre::MaterialManager::getSingleton().parseScript(
                  stream, "General");
              Ogre::MaterialPtr matPtr =
                Ogre::MaterialManager::getSingleton().getByName(
                    fullPath.string());

              if (!matPtr.isNull())
              {
                // is this necessary to do here? Someday try it without
                matPtr->compile();
                matPtr->load();
              }
            }
            catch(Ogre::Exception& e)
            {
              gzerr << "Unable to parse material file[" << fullPath << "]\n";
            }
            stream->close();
          }
        }
      }
    }
  }
  catch(Ogre::Exception &/*_e*/)
  {
    gzthrow("Unable to load Ogre Resources.\nMake sure the "
        "resources path in the world file is set correctly.");
  }
}

//////////////////////////////////////////////////
RenderEngine::RenderPathType RenderEngine::GetRenderPathType() const
{
  return this->dataPtr->renderPathType;
}

//////////////////////////////////////////////////
void RenderEngine::SetupResources()
{
  std::vector< std::pair<std::string, std::string> > archNames;
  std::vector< std::pair<std::string, std::string> >::iterator aiter;
  std::list<std::string>::const_iterator iter;
  std::list<std::string> paths =
    common::SystemPaths::Instance()->GetGazeboPaths();

  std::list<std::string> mediaDirs;
  mediaDirs.push_back("media");

  for (iter = paths.begin(); iter != paths.end(); ++iter)
  {
    DIR *dir;
    if ((dir = opendir((*iter).c_str())) == NULL)
    {
      continue;
    }
    closedir(dir);

    archNames.push_back(
        std::make_pair((*iter)+"/", "General"));

    for (std::list<std::string>::iterator mediaIter = mediaDirs.begin();
         mediaIter != mediaDirs.end(); ++mediaIter)
    {
      std::string prefix = (*iter) + "/" + (*mediaIter);

      archNames.push_back(
          std::make_pair(prefix, "General"));
      archNames.push_back(
          std::make_pair(prefix + "/skyx", "SkyX"));
#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0) && !defined(__APPLE__))
      archNames.push_back(
          std::make_pair(prefix + "/rtshaderlib150", "General"));
#endif
      archNames.push_back(
          std::make_pair(prefix + "/rtshaderlib", "General"));
#if (OGRE_RESOURCEMANAGER_STRICT == 0)
      archNames.push_back(
          std::make_pair(prefix + "/materials/programs", "General"));
#endif
      archNames.push_back(
          std::make_pair(prefix + "/materials/scripts", "General"));
      archNames.push_back(
          std::make_pair(prefix + "/materials/textures", "General"));
      archNames.push_back(
          std::make_pair(prefix + "/media/models", "General"));
      archNames.push_back(
          std::make_pair(prefix + "/fonts", "Fonts"));
      archNames.push_back(
          std::make_pair(prefix + "/gui/looknfeel", "LookNFeel"));
      archNames.push_back(
          std::make_pair(prefix + "/gui/schemes", "Schemes"));
      archNames.push_back(
          std::make_pair(prefix + "/gui/imagesets", "Imagesets"));
      archNames.push_back(
          std::make_pair(prefix + "/gui/fonts", "Fonts"));
      archNames.push_back(
          std::make_pair(prefix + "/gui/layouts", "Layouts"));
      archNames.push_back(
          std::make_pair(prefix + "/gui/animations", "Animations"));
#if (OGRE_RESOURCEMANAGER_STRICT != 0)
    try
    {
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          boost::filesystem::path(
              prefix + "/materials/programs").make_preferred().string(),
          "FileSystem", "General", true);
    }
    catch(Ogre::Exception &/*_e*/)
    {
      gzerr << "Unable to load Ogre Resources. Make sure the resources path "
            << "in the world file is set correctly.";
    }
#endif
    }
  }

  for (aiter = archNames.begin(); aiter != archNames.end(); ++aiter)
  {
    try
    {
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          boost::filesystem::path(aiter->first).make_preferred().string(),
          "FileSystem", aiter->second);
    }
    catch(Ogre::Exception &/*_e*/)
    {
      gzthrow("Unable to load Ogre Resources. Make sure the resources path "
          "in the world file is set correctly.");
    }
  }
}

//////////////////////////////////////////////////
void RenderEngine::SetupRenderSystem()
{
  Ogre::RenderSystem *renderSys;
  const Ogre::RenderSystemList *rsList;

  // Set parameters of render system (window size, etc.)
#if  OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
  rsList = this->dataPtr->root->getAvailableRenderers();
#else
  rsList = &(this->dataPtr->root->getAvailableRenderers());
#endif

  int c = 0;

  renderSys = NULL;

  do
  {
    if (c == static_cast<int>(rsList->size()))
      break;

    renderSys = rsList->at(c);
    c++;
  }
  while (renderSys &&
         renderSys->getName().compare("OpenGL Rendering Subsystem") != 0);

  if (renderSys == NULL)
  {
    gzthrow("unable to find OpenGL rendering system. OGRE is probably "
            "installed incorrectly. Double check the OGRE cmake output, "
            "and make sure OpenGL is enabled.");
  }

  // We operate in windowed mode
  renderSys->setConfigOption("Full Screen", "No");

  /// We used to allow the user to set the RTT mode to PBuffer, FBO, or Copy.
  ///   Copy is slow, and there doesn't seem to be a good reason to use it
  ///   PBuffer limits the size of the renderable area of the RTT to the
  ///           size of the first window created.
  ///   FBO seem to be the only good option
  renderSys->setConfigOption("RTT Preferred Mode", "FBO");

  // get all supported fsaa values
  Ogre::ConfigOptionMap configMap = renderSys->getConfigOptions();
  auto fsaaOoption = configMap.find("FSAA");

  if (fsaaOoption != configMap.end())
  {
    auto values = (*fsaaOoption).second.possibleValues;
    for (auto const &str : values)
    {
      int value = 0;
      try
      {
        value = std::stoi(str);
      }
      catch(...)
      {
        continue;
      }
      this->dataPtr->fsaaLevels.push_back(value);
    }
  }
  std::sort(this->dataPtr->fsaaLevels.begin(), this->dataPtr->fsaaLevels.end());

  // check if target fsaa is supported
  unsigned int fsaa = 0;
  unsigned int targetFSAA = 4;
  auto const it = std::find(this->dataPtr->fsaaLevels.begin(),
      this->dataPtr->fsaaLevels.end(), targetFSAA);
  if (it != this->dataPtr->fsaaLevels.end())
    fsaa = targetFSAA;

  renderSys->setConfigOption("FSAA", std::to_string(fsaa));

  this->dataPtr->root->setRenderSystem(renderSys);
}

/////////////////////////////////////////////////
bool RenderEngine::CreateContext()
{
  bool result = true;

#if defined __APPLE__ || _WIN32
  this->dummyDisplay = 0;
#else
  try
  {
    this->dummyDisplay = XOpenDisplay(0);
    if (!this->dummyDisplay)
    {
      gzerr << "Can't open display: " << XDisplayName(0) << "\n";
      return false;
    }

    int screen = DefaultScreen(this->dummyDisplay);

    int attribList[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, 16,
      GLX_STENCIL_SIZE, 8, None };

    XVisualInfo *dummyVisual = glXChooseVisual(
        static_cast<Display*>(this->dummyDisplay),
        screen, static_cast<int *>(attribList));

    if (!dummyVisual)
    {
      gzerr << "Unable to create glx visual\n";
      return false;
    }

    this->dummyWindowId = XCreateSimpleWindow(
        static_cast<Display*>(this->dummyDisplay),
        RootWindow(static_cast<Display*>(this->dummyDisplay), screen),
        0, 0, 1, 1, 0, 0, 0);

    this->dummyContext = glXCreateContext(
        static_cast<Display*>(this->dummyDisplay),
        dummyVisual, NULL, 1);

    if (!this->dummyContext)
    {
      gzerr << "Unable to create glx context\n";
      return false;
    }

    glXMakeCurrent(static_cast<Display*>(this->dummyDisplay),
        this->dummyWindowId, static_cast<GLXContext>(this->dummyContext));
  }
  catch(...)
  {
    result = false;
  }
#endif

  return result;
}

/////////////////////////////////////////////////
void RenderEngine::CheckSystemCapabilities()
{
  const Ogre::RenderSystemCapabilities *capabilities;
  Ogre::RenderSystemCapabilities::ShaderProfiles profiles;
  Ogre::RenderSystemCapabilities::ShaderProfiles::const_iterator iter;

  capabilities = this->dataPtr->root->getRenderSystem()->getCapabilities();
  if (!capabilities)
  {
    gzerr << "Cannot get render system capabilities" << std::endl;
    return;
  }

  profiles = capabilities->getSupportedShaderProfiles();

  bool hasFragmentPrograms =
    capabilities->hasCapability(Ogre::RSC_FRAGMENT_PROGRAM);

  bool hasVertexPrograms =
    capabilities->hasCapability(Ogre::RSC_VERTEX_PROGRAM);

  // bool hasGeometryPrograms =
  //  capabilities->hasCapability(Ogre::RSC_GEOMETRY_PROGRAM);

  // bool hasRenderToVertexBuffer =
  //  capabilities->hasCapability(Ogre::RSC_HWRENDER_TO_VERTEX_BUFFER);

  // int multiRenderTargetCount = capabilities->getNumMultiRenderTargets();

  bool hasFBO =
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 11
    // All APIs targeted by OGRE supported this capability,
    // see https://ogrecave.github.io/ogre/api/1.10/group___render_system.html#gga3d2965b7f378ebdcfe8a4a6cf74c3de7a8a0ececdc95122ac3063fc4f27d6402c
    true;
#else
    capabilities->hasCapability(Ogre::RSC_FBO);
#endif

  bool hasGLSL =
    std::find(profiles.begin(), profiles.end(), "glsl") != profiles.end();

  if (!hasFragmentPrograms || !hasVertexPrograms)
    gzwarn << "Vertex and fragment shaders are missing. "
           << "Fixed function rendering will be used.\n";

  if (!hasGLSL)
    gzwarn << "GLSL is missing. "
           << "Fixed function rendering will be used.\n";

  if (!hasFBO)
    gzwarn << "Frame Buffer Objects (FBO) is missing. "
           << "Rendering will be disabled.\n";

  this->dataPtr->renderPathType = RenderEngine::NONE;

  if (hasFBO && hasGLSL && hasVertexPrograms && hasFragmentPrograms)
    this->dataPtr->renderPathType = RenderEngine::FORWARD;
  else if (hasFBO)
    this->dataPtr->renderPathType = RenderEngine::VERTEX;

  // Disable deferred rendering for now. Needs more work.
  // if (hasRenderToVertexBuffer && multiRenderTargetCount >= 8)
  //  this->dataPtr->renderPathType = RenderEngine::DEFERRED;
}

/////////////////////////////////////////////////
WindowManagerPtr RenderEngine::GetWindowManager() const
{
  return this->dataPtr->windowManager;
}

/////////////////////////////////////////////////
Ogre::Root *RenderEngine::Root() const
{
  return this->dataPtr->root;
}

/////////////////////////////////////////////////
std::vector<unsigned int> RenderEngine::FSAALevels() const
{
  return this->dataPtr->fsaaLevels;
}

#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
/////////////////////////////////////////////////
Ogre::OverlaySystem *RenderEngine::OverlaySystem() const
{
  return this->dataPtr->overlaySystem;
}
#endif

//////////////////////////////////////////////////
RenderEngine* RenderEngine::Instance()
{
#ifndef _WIN32
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  return SingletonT<RenderEngine>::Instance();
#ifndef _WIN32
  #pragma GCC diagnostic pop
#endif
}
