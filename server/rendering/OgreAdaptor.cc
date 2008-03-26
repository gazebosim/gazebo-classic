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

#include "MovableText.hh"
#include "OgreHUD.hh"
#include "Entity.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "Global.hh"
#include "GazeboError.hh"
#include "XMLConfig.hh"
#include "Simulator.hh"
#include "Gui.hh"
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

  this->ogreWindow = true;
  this->window=NULL;
  this->backgroundColor=NULL;
  this->logManager=NULL;
  this->sceneMgr=NULL;
  this->camera=NULL;
  this->viewport=NULL;
  this->root=NULL;

}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
OgreAdaptor::~OgreAdaptor()
{
  //GZ_DELETE (this->window)
  //GZ_DELETE (this->backgroundColor)
  //GZ_DELETE (this->frameListener)
  //GZ_DELETE (this->logManager)
//  this->root->detachRenderTarget(this->window);
//  this->root->shutdown();
  //GZ_DELETE (this->root)
//  GZ_DELETE (this->sceneMgr) //this objects seems to be destroyed by root
//  GZ_DELETE (this->camera)
//  GZ_DELETE (this->viewport)

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
// Init
void OgreAdaptor::Init(XMLConfigNode *rootNode)
{
  XMLConfigNode *cnode;
  XMLConfigNode *node;
  Ogre::ColourValue ambient;

  try
  {
    // Make the root
    this->root = new Ogre::Root();
  }
  catch (Ogre::Exception e)
  {
    gzthrow("Unable to create an Ogre rendering environment, no Root ");
    exit(0);
  }


  node = rootNode->GetChild("ogre", "rendering");
  if (!node)
  {
    gzthrow( "missing OGRE Rendering information" );
  }

  ambient.r = node->GetTupleDouble("ambient",0,1.0);
  ambient.g = node->GetTupleDouble("ambient",1,1.0);
  ambient.b = node->GetTupleDouble("ambient",2,1.0);
  ambient.a = node->GetTupleDouble("ambient",3,1.0);

  if ((cnode=node->GetChild("video")))
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

  // Default background color
  this->backgroundColor = new Ogre::ColourValue(Ogre::ColourValue::Black);

  // Load all the plugins
  this->LoadPlugins();

  // Setup the available resources
  this->SetupResources();

  // Setup the rendering system, and create the context
  this->SetupRenderSystem(true);

  // Initialize the root node, and don't create a window
  this->window = this->root->initialise(false);

  // Create a window for Ogre
  this->CreateWindow();

  // Get the SceneManager, in this case a generic one
  if (node->GetChild("bsp"))
  {
    this->sceneMgr = this->root->createSceneManager("BspSceneManager");
    this->sceneType= SCENE_BSP;
  }
  else
  {
    this->sceneMgr = this->root->createSceneManager(Ogre::ST_EXTERIOR_CLOSE);
    this->sceneType= SCENE_EXT;
  }

  // Set default mipmap level (NB some APIs ignore this)
  Ogre::TextureManager::getSingleton().setDefaultNumMipmaps( 5 );

  // Load Resources
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  // Ambient lighting
  this->sceneMgr->setAmbientLight(ambient);

  // Settings for shadow mapping
  std::string shadowTechnique = node->GetString("shadowTechnique", "stencilAdditive");
  if (shadowTechnique == std::string("stencilAdditive"))
    this->sceneMgr->setShadowTechnique( Ogre::SHADOWTYPE_STENCIL_ADDITIVE );
  else if (shadowTechnique == std::string("textureAdditive"))
    this->sceneMgr->setShadowTechnique( Ogre::SHADOWTYPE_TEXTURE_ADDITIVE );
  else if (shadowTechnique == std::string("none"))
    this->sceneMgr->setShadowTechnique( Ogre::SHADOWTYPE_NONE );
  else gzthrow(std::string("Unsupported shadow technique: ") + shadowTechnique + "\n");

  this->sceneMgr->setShadowTextureSelfShadow(true);
  this->sceneMgr->setShadowTextureSize(node->GetInt("shadowTextureSize", 512));
  this->sceneMgr->setShadowIndexBufferSize( node->GetInt("shadowIndexSize",this->sceneMgr->getShadowIndexBufferSize()) );


  // Add fog. This changes the background color
  OgreCreator::CreateFog(node);

  // Add a sky dome to our scene
  OgreCreator::CreateSky(node);

  // Create our frame listener and register it
  this->frameListener = new OgreFrameListener();
  this->root->addFrameListener(this->frameListener);

  // Create the default camera. This camera is only used to view the output
  // of cameras created using the XML world file
  this->camera = this->sceneMgr->createCamera("__GAZEBO_CAMERA__");
  this->camera->setNearClipDistance(1);
  this->camera->setFarClipDistance(1);
  this->viewport = this->window->addViewport(this->camera);
  this->viewport->setBackgroundColour(Ogre::ColourValue::Black);

  this->camera->setAspectRatio( Ogre::Real(viewport->getActualWidth()) / Ogre::Real(viewport->getActualHeight()) );

  if (node->GetBool("grid", true))
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
      gzmsg(0) << "Unable to load BSP geometry." << e.getDescription() << "\n";
      exit(0);
    }
  }

  /*
    Ogre::ManualObject* myManualObject =  this->sceneMgr->createManualObject("manual1");
    Ogre::SceneNode* myManualObjectNode = this->sceneMgr->getRootSceneNode()->createChildSceneNode("manual1_node");

    Ogre::MaterialPtr myManualObjectMaterial = Ogre::MaterialManager::getSingleton().create("manual1Material","debugger");
    myManualObjectMaterial->setReceiveShadows(false);
    myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true);
    myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,0,1,0);
    myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(0,0,1);
    myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,1);

    myManualObject->begin("manual1Material", Ogre::RenderOperation::OT_LINE_LIST);
    myManualObject->position(0, 0, 0);
    myManualObject->position(0, 1, 0);
    myManualObject->position(0, 0, 0);
    myManualObject->position(0, 0, 1);
    myManualObject->position(0, 0, 0);
    myManualObject->position(1, 0, 0);
    // etc
    myManualObject->end();
    myManualObjectNode->attachObject(myManualObject);
    */

  //delete [] mstr;
  //

}


void OgreAdaptor::Save(XMLConfigNode *node)
{
  //Video information is not modified so we don't need to rewrite it.
  //Sky is not modified, not rewritten
  XMLConfigNode *rnode;
  XMLConfigNode *cnode;

  rnode = node->GetChild("ogre", "rendering");
  if (!rnode)
    gzthrow( "missing OGRE Rendering information, can't write back the data" );

  rnode->SetValue("ambient", this->backgroundColor);
  //TODO: BSP (when bsp are definitely integrated)

  if (cnode=node->GetChild("fog"))
    OgreCreator::SaveFog(cnode);

}


////////////////////////////////////////////////////////////////////////////////
// Load plugins
void OgreAdaptor::LoadPlugins()
{
  std::string pathStr;
  std::string pluginStr;
  XMLConfigNode *pluginNode;
  std::list<std::string>::iterator iter;

  for (iter=Global::ogrePaths.begin(); iter!=Global::ogrePaths.end(); iter++)
  {
    DIR *dir;
    if ((dir=opendir((*iter).c_str())) == NULL)
    {
      continue;
    }
    closedir(dir);

    std::vector<std::string> plugins;
    std::vector<std::string>::iterator piter;

    plugins.push_back((*iter)+"/RenderSystem_GL.so");
    plugins.push_back((*iter)+"/Plugin_ParticleFX.so");
    plugins.push_back((*iter)+"/Plugin_BSPSceneManager.so");
    plugins.push_back((*iter)+"/Plugin_OctreeSceneManager.so");

    for (piter=plugins.begin(); piter!=plugins.end(); piter++)
    {
      try
      {
        // Load the plugin into OGRE
        this->root->loadPlugin(*piter);
      }
      catch (Ogre::Exception e)
      {
        gzthrow("Unable to load Ogre Plugins.\nMake sure the plugins path in the world file is set correctly");
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

  for (iter=Global::gazeboPaths.begin();
       iter!=Global::gazeboPaths.end(); iter++)
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
  // Set parameters of render system (window size, etc.)
  //if (!this->root->restoreConfig())
  {
    Ogre::RenderSystemList *rsList = this->root->getAvailableRenderers();
    int c = 0;
    Ogre::RenderSystem *selectedRenderSystem = NULL;

    do
    {
      if (c == (int)rsList->size())
        break;

      selectedRenderSystem = rsList->at(c);
      c++;
    }
    while (selectedRenderSystem->getName().compare("OpenGL Rendering Subsystem")!= 0);

    if (selectedRenderSystem == NULL)
    {
      gzthrow( "unable to find rendering system" );
    }


    selectedRenderSystem->setConfigOption("Full Screen","No");
    selectedRenderSystem->setConfigOption("FSAA","2");

    // Set the preferred RRT mode. Options are: "PBuffer", "FBO", and "Copy", can be set in the .gazeborc file
    selectedRenderSystem->setConfigOption("RTT Preferred Mode", Global::RTTMode);

    if (create && this->videoMode != "None")
    {
      selectedRenderSystem->setConfigOption("Video Mode",this->videoMode);
      this->root->setRenderSystem(selectedRenderSystem);
    }
    else
    {
      std::cerr << "No render system selected\n";
    }

  }
}


////////////////////////////////////////////////////////////////////////////////
// Create a window for Ogre
void OgreAdaptor::CreateWindow()
{
  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;
  Gui *gui=Simulator::Instance()->GetUI();

  paramsVector.push_back( Ogre::StringConverter::toString( (size_t)(gui->GetDisplay()) ) );
  paramsVector.push_back( Ogre::StringConverter::toString((int)gui->GetVisualInfo()->screen));

  paramsVector.push_back( Ogre::StringConverter::toString((int)gui->GetWindowId()));
  paramsVector.push_back( Ogre::StringConverter::toString((size_t)(gui->GetVisualInfo())));

  params["parentWindowHandle"] = Ogre::StringConverter::toString(paramsVector);

  this->window = this->root->createRenderWindow( "WindowName", gui->GetWidth(), gui->GetHeight(), false, &params);

  this->window->setActive(true);
  this->window->setAutoUpdated(true);
}

////////////////////////////////////////////////////////////////////////////////
// Resize the render window
void OgreAdaptor::ResizeWindow(unsigned int w, unsigned int h)
{
  this->window->resize(w, h);
  this->window->update();

  this->viewport->setDimensions(0,0,1,1);
  this->camera->setAspectRatio( Ogre::Real(viewport->getActualWidth()) / Ogre::Real(viewport->getActualHeight()) );
  //this->frameListener->Resize(w,h);
}


////////////////////////////////////////////////////////////////////////////////
// Render
int OgreAdaptor::Render()
{
  OgreHUD::Instance()->Update();

  root->renderOneFrame();
  return 0;
}

