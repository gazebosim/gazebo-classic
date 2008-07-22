/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Middleman between OGRE and Gazebo
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * CVS: $Id$
 */

#include <Ogre.h>
#include <OgreDataStream.h>
#include <OgreLogManager.h>
#include <OgreWindowEventUtilities.h>

#include <sys/types.h>
#include <dirent.h>
#include <iostream>
#include <string.h>

#include "Model.hh"
#include "OgreVisual.hh"
#include "UserCamera.hh"
#include "MovableText.hh"
#include "OgreHUD.hh"
#include "Entity.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "GazeboConfig.hh"
#include "Global.hh"
#include "XMLConfig.hh"
#include "Simulator.hh"
#include "OgreFrameListener.hh"
#include "OgreCreator.hh"
#include "OgreAdaptor.hh"

using namespace gazebo;

enum SceneTypes{SCENE_BSP, SCENE_EXT};


////////////////////////////////////////////////////////////////////////////////
/// Constructor
OgreAdaptor::OgreAdaptor()
{
  // Create a new log manager and prevent output from going to stdout
  this->logManager = new Ogre::LogManager();
  this->logManager->createLog("Ogre.log", true, false, false);

  this->backgroundColor=NULL;
  this->logManager=NULL;
  this->sceneMgr=NULL;
  this->root=NULL;

  this->updateRate = 0;

  this->dummyDisplay = false;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
OgreAdaptor::~OgreAdaptor()
{
  //GZ_DELETE (this->backgroundColor)
  //GZ_DELETE (this->frameListener)
  //GZ_DELETE (this->logManager)
//  this->root->shutdown();
  //GZ_DELETE (this->root)
//  GZ_DELETE (this->sceneMgr) //this objects seems to be destroyed by root
//  GZ_DELETE (this->viewport)

  if (this->dummyDisplay)
  {
    glXDestroyContext(this->dummyDisplay, this->dummyContext);
    XDestroyWindow(this->dummyDisplay, this->dummyWindowId);
    XCloseDisplay(this->dummyDisplay);
  }

}

////////////////////////////////////////////////////////////////////////////////
// Closes and free
void OgreAdaptor::Close()
{
  GZ_DELETE (this->frameListener)

  // This causes a seg fault. Need to fix
  //GZ_DELETE (this->root)
}

////////////////////////////////////////////////////////////////////////////////
/// Load the parameters for Ogre
void OgreAdaptor::Load(XMLConfigNode *rootNode)
{
  XMLConfigNode *cnode;
  XMLConfigNode *node;

  node = rootNode->GetChild("ogre", "rendering");

  // Make the root
  try
  {
    this->root = new Ogre::Root();
  }
  catch (Ogre::Exception e)
  {
    gzthrow("Unable to create an Ogre rendering environment, no Root ");
  }

  // Default background color
  this->backgroundColor = new Ogre::ColourValue(Ogre::ColourValue::Black);

  // Load all the plugins
  this->LoadPlugins();

  // Setup the available resources
  this->SetupResources();

  if ((cnode = node->GetChild("video")))
  {
    std::ostringstream stream;
    int width, height, depth;

    width = cnode->GetTupleInt("size",0,800);
    height = cnode->GetTupleInt("size",1,600);
    depth = cnode->GetInt("depth",16,0);

    stream << width << " x " << height << " @ " << depth << "-bit colour";
    this->videoMode = stream.str();
  }
  else
  {
    this->videoMode = "800 x 600 @ 16-bit colour";
  }

  this->videoMode = "800 x 600 @ 16-bit colour";

  // Setup the rendering system, and create the context
  this->SetupRenderSystem(true);

  // Initialize the root node, and don't create a window
  this->root->initialise(false);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize ogre
void OgreAdaptor::Init(XMLConfigNode *rootNode)
{
  XMLConfigNode *node;
  Ogre::ColourValue ambient;

  node = rootNode->GetChild("ogre", "rendering");

  /// Create a dummy rendering context if the GUI is disabled
  if (!Simulator::Instance()->GetGuiEnabled())
  {
    this->dummyDisplay = XOpenDisplay(0);
    if (!this->dummyDisplay) 
      gzthrow(std::string("Can't open display: ") + XDisplayName(0) + "\n");

    int screen = DefaultScreen(this->dummyDisplay);

    int attribList[8] = {GLX_RGBA, GLX_RED_SIZE, 8, GLX_GREEN_SIZE, 8,
                         GLX_BLUE_SIZE, 8,	None};

    this->dummyVisual = glXChooseVisual(this->dummyDisplay, screen, 
                                        (int *)attribList);

    this->dummyWindowId = XCreateSimpleWindow(this->dummyDisplay, 
        RootWindow(this->dummyDisplay, screen), 0, 0, 1, 1, 0, 0, 0);

    this->dummyContext = glXCreateContext(this->dummyDisplay, 
                                          this->dummyVisual, NULL, 1);

    glXMakeCurrent(this->dummyDisplay, this->dummyWindowId, this->dummyContext);
    OgreCreator::CreateWindow((long)this->dummyDisplay, screen, 
                              (long)this->dummyWindowId,1,1);
  }

  // Set default mipmap level (NB some APIs ignore this)
  Ogre::TextureManager::getSingleton().setDefaultNumMipmaps( 5 );

  // Load Resources
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  // Get the SceneManager, in this case a generic one
  if (node->GetChild("bsp"))
  {
    this->sceneType= SCENE_BSP;
    this->sceneMgr = this->root->createSceneManager("BspSceneManager");
  }
  else
  {
    this->sceneType= SCENE_EXT;
    //this->sceneMgr = this->root->createSceneManager(Ogre::ST_EXTERIOR_CLOSE);
    this->sceneMgr = this->root->createSceneManager(Ogre::ST_EXTERIOR_FAR);
  }

  ambient.r = node->GetTupleDouble("ambient",0,1.0);
  ambient.g = node->GetTupleDouble("ambient",1,1.0);
  ambient.b = node->GetTupleDouble("ambient",2,1.0);
  ambient.a = node->GetTupleDouble("ambient",3,1.0);

  // Ambient lighting
  this->sceneMgr->setAmbientLight(ambient);

  this->sceneMgr->setShadowTextureSelfShadow(true);
  this->sceneMgr->setShadowTexturePixelFormat(Ogre::PF_FLOAT32_R);
  this->sceneMgr->setShadowTextureSize(node->GetInt("shadowTextureSize", 512));
  this->sceneMgr->setShadowIndexBufferSize( node->GetInt("shadowIndexSize",this->sceneMgr->getShadowIndexBufferSize()) );

   // Settings for shadow mapping
  std::string shadowTechnique = node->GetString("shadowTechnique", "stencilAdditive");

  if (shadowTechnique == std::string("stencilAdditive"))
    this->sceneMgr->setShadowTechnique( Ogre::SHADOWTYPE_STENCIL_ADDITIVE );
  else if (shadowTechnique == std::string("textureAdditive"))
    this->sceneMgr->setShadowTechnique( Ogre::SHADOWTYPE_TEXTURE_ADDITIVE );
  else if (shadowTechnique == std::string("none"))
    this->sceneMgr->setShadowTechnique( Ogre::SHADOWTYPE_NONE );
  else 
    gzthrow(std::string("Unsupported shadow technique: ") + shadowTechnique + "\n");

  //Preload basic shapes that can be used anywhere
  OgreCreator::LoadBasicShapes();

  this->fogNode = node->GetChild("fog");
  this->skyNode = node->GetChild("sky");
  this->drawGrid = node->GetBool("grid", true);

  // Add a sky dome to our scene
  OgreCreator::CreateSky(this->skyNode);

  // Add fog. This changes the background color
  OgreCreator::CreateFog(this->fogNode);

  if (this->drawGrid)
    OgreCreator::DrawGrid();

  // Set up the world geometry link
  if (this->sceneType==SCENE_BSP)
  {
    this->worldGeometry = node->GetString("bsp","",1);

    try
    {
      this->sceneMgr->setWorldGeometry(this->worldGeometry);
    }
    catch (Ogre::Exception e)
    {
      gzmsg(-1) << "Unable to load BSP geometry." << e.getDescription() << "\n";
      exit(-1);
    }
  }
  // Create our frame listener and register it
  this->frameListener = new OgreFrameListener();
  this->root->addFrameListener(this->frameListener);

  this->updateRate = node->GetDouble("maxUpdateRate",0,0);

  this->raySceneQuery = this->sceneMgr->createRayQuery( Ogre::Ray() );
  this->raySceneQuery->setSortByDistance(true);
  this->raySceneQuery->setQueryMask(Ogre::SceneManager::ENTITY_TYPE_MASK);
}

////////////////////////////////////////////////////////////////////////////////
// Save
void OgreAdaptor::Save(XMLConfigNode *node)
{
  /*
  //Video information is not modified so we don't need to rewrite it.
  //Sky is not modified, not rewritten
  XMLConfigNode *rnode;
  XMLConfigNode *cnode;

  rnode = node->GetChild("ogre", "rendering");
  if (!rnode)
    gzthrow( "missing OGRE Rendering information, can't write back the data" );

  rnode->SetValue("ambient", this->backgroundColor);
  //TODO: BSP (when bsp are definitely integrated)
  //
  cnode = rnode->GetChild("fog");
  if (cnode)
    OgreCreator::SaveFog(cnode);
    */
}


////////////////////////////////////////////////////////////////////////////////
// Load plugins
void OgreAdaptor::LoadPlugins()
{
  std::list<std::string>::iterator iter;
  std::list<std::string> ogrePaths=Simulator::Instance()->GetGazeboConfig()->GetOgrePaths();
 
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

    for (piter=plugins.begin(); piter!=plugins.end(); piter++)
    {
      try
      {
        // Load the plugin into OGRE
        this->root->loadPlugin(*piter);
        //gzmsg(2) << "Loaded plugin:" << (*piter);
      }
      catch (Ogre::Exception e)
      {
        std::string description("Unable to load Ogre Plugins on directory ");
        description.append(path);
        description.append("\n Make sure the plugins path in the gazebo configuration file are set correctly.\n");
        gzthrow( description + e.getDescription() );
      }
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
// Setup resources
void OgreAdaptor::SetupResources()
{
  std::vector<std::string> archNames;
  std::vector<std::string>::iterator aiter;
  std::list<std::string>::iterator iter;

  for (iter=Simulator::Instance()->GetGazeboConfig()->GetGazeboPaths().begin();
       iter!=Simulator::Instance()->GetGazeboConfig()->GetGazeboPaths().end(); iter++)
  {
    DIR *dir;
    if ((dir=opendir((*iter).c_str())) == NULL)
    {
      continue;
    }
    closedir(dir);

    archNames.push_back((*iter)+"/Media");
    archNames.push_back((*iter)+"/Media/fonts");
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
        //std::cout << filename << std::endl;
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
void OgreAdaptor::SetupRenderSystem(bool create)
{

  Ogre::RenderSystem *renderSys;

  // Set parameters of render system (window size, etc.)
  //if (!this->root->restoreConfig())
  {
    Ogre::RenderSystemList *rsList = this->root->getAvailableRenderers();
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
    {
      gzthrow( "unable to find rendering system" );
    }


    renderSys->setConfigOption("Full Screen","No");
    renderSys->setConfigOption("FSAA","2");

    // Set the preferred RRT mode. Options are: "PBuffer", "FBO", and "Copy", can be set in the .gazeborc file
    renderSys->setConfigOption("RTT Preferred Mode", Simulator::Instance()->GetGazeboConfig()->GetRTTMode());

    if (create && this->videoMode != "None")
    {
      renderSys->setConfigOption("Video Mode",this->videoMode);
      this->root->setRenderSystem(renderSys);
    }
    else
    {
      std::cerr << "No render system selected\n";
    }

  }
}


////////////////////////////////////////////////////////////////////////////////
// Update a window
void OgreAdaptor::UpdateWindow(Ogre::RenderWindow *window, OgreCamera *camera)
{
  this->root->_fireFrameStarted();

  window->update();

  this->root->_fireFrameEnded();
}

////////////////////////////////////////////////////////////////////////////////
/// Get an entity at a pixel location using a camera. Used for mouse picking. 
Entity *OgreAdaptor::GetEntityAt(OgreCamera *camera, Vector2<int> mousePos) 
{
  Entity *entity = NULL;
  Ogre::Camera *ogreCam = camera->GetOgreCamera();
  Ogre::Vector3 camPos = ogreCam->getPosition();

  Ogre::Ray mouseRay = ogreCam->getCameraToViewportRay(
      (float)mousePos.x / ogreCam->getViewport()->getActualWidth(), 
      (float)mousePos.y / ogreCam->getViewport()->getActualHeight() );

  this->raySceneQuery->setRay( mouseRay );

  // Perform the scene query
  Ogre::RaySceneQueryResult &result = this->raySceneQuery->execute();
  Ogre::RaySceneQueryResult::iterator iter = result.begin();

  for (iter = result.begin(); iter != result.end(); iter++)
  {
    if (iter->movable)
    {

      OgreVisual *vis = dynamic_cast<OgreVisual*>(iter->movable->getUserObject());
      if (vis && vis->GetEntity())
      {
        entity = vis->GetEntity();
        entity->GetVisualNode()->ShowSelectionBox(true);
        Model *model = NULL;
        
        do 
        {
          model = dynamic_cast<Model*>(entity);
          entity = entity->GetParent();
        } while (model == NULL);

        return model;
      }
    }

  }

  return NULL;
}


////////////////////////////////////////////////////////////////////////////////
/// Get the desired update rate
double OgreAdaptor::GetUpdateRate()
{
  return this->updateRate;
}
