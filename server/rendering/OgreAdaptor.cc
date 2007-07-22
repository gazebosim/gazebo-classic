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
#include <OgreLogManager.h>
#include <OgreWindowEventUtilities.h>

#include "OgreHUD.hh"
#include "Entity.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "Global.hh"
#include "GazeboError.hh"
#include "XMLConfig.hh"
#include "OgreFrameListener.hh"
#include "OgreAdaptor.hh"

using namespace gazebo;

OgreAdaptor *OgreAdaptor::myself = NULL;

/// Constructor
OgreAdaptor::OgreAdaptor()
{
  // Create a new log manager and prevent output from going to stdout
  this->logManager = new Ogre::LogManager();
  this->logManager->createLog("Ogre.log", true, false, false);

  // Make the root 
  this->root = new Ogre::Root(); 
}

/// Destructor
OgreAdaptor::~OgreAdaptor()
{
}

OgreAdaptor *OgreAdaptor::Instance()
{
  if (!myself)
    myself = new OgreAdaptor;

  return myself;
}


void OgreAdaptor::Init(XMLConfigNode *node)
{
  XMLConfigNode *cnode;
  Ogre::ColourValue ambient;

  if (!node)
  {
    gzthrow( "missing OGRE Rendernig information" );
  }

  ambient.r = node->GetTupleDouble("ambient",0,0);
  ambient.g = node->GetTupleDouble("ambient",1,0);
  ambient.b = node->GetTupleDouble("ambient",2,0);
  ambient.a = node->GetTupleDouble("ambient",3,0);

  if ((cnode=node->GetChild("video")))
  {
    std::ostringstream stream;
    int width, height, depth;

    width = cnode->GetTupleInt("size",0,640);
    height = cnode->GetTupleInt("size",1,480);
    depth = cnode->GetInt("depth",16,0);

    stream << width << " x " << height << " @ " << depth << "-bit colour";
    this->videoMode = stream.str();
  }
  else
  {
    this->videoMode = "640 x 480 @ 16-bit colour";
  }

  // Default background color
  this->backgroundColor = new Ogre::ColourValue(Ogre::ColourValue::Black);

  // Load all the plugins
  this->LoadPlugins(node->GetChild("plugins"));

  // Setup the available resources 
  this->SetupResources(node->GetChild("resources"));

  // Setup the rendering system, and create the context
  this->SetupRenderSystem(true);

  // Initialize the root node, and create a window
  this->window = this->root->initialise(true); 

  // Get the SceneManager, in this case a generic one
  this->sceneMgr = this->root->createSceneManager(Ogre::ST_GENERIC);
  //this->sceneMgr = this->root->createSceneManager(Ogre::ST_EXTERIOR_CLOSE);

  // Set default mipmap level (NB some APIs ignore this)
  Ogre::TextureManager::getSingleton().setDefaultNumMipmaps( 5 );

  // Load Resources
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  // Get the SceneManager, in this case a generic one
  // Default lighting
  this->sceneMgr->setAmbientLight(ambient); 
  this->sceneMgr->setShadowTechnique( Ogre::SHADOWTYPE_STENCIL_ADDITIVE );

  // Add a sky dome to our scene
  if ((cnode = node->GetChild("sky")))
  {
    this->sceneMgr->setSkyDome(true,cnode->GetString("material","",0),5,8);
  }

  // Add fog. This changes the background color
  if ((cnode = node->GetChild("fog")))
  {
    Ogre::FogMode fogType = Ogre::FOG_NONE; 
    std::string type;
    double density, linearStart, linearEnd;

    this->backgroundColor->r = cnode->GetTupleDouble("color",0,0);
    this->backgroundColor->g = cnode->GetTupleDouble("color",1,0);
    this->backgroundColor->b = cnode->GetTupleDouble("color",2,0);

    type = cnode->GetString("type","linear",0);
    density = cnode->GetDouble("density",0,0);
    linearStart = cnode->GetDouble("linearStart",0,0);
    linearEnd = cnode->GetDouble("linearEnd",1.0,0);

    if (type == "linear")
      fogType = Ogre::FOG_LINEAR;
    else if (type == "exp")
      fogType = Ogre::FOG_EXP;
    else if (type == "exp2")
      fogType = Ogre::FOG_EXP2;

    //this->sceneMgr->setFog(fogType, *this->backgroundColor, density, linearStart, linearEnd);
    this->sceneMgr->setFog(Ogre::FOG_LINEAR, *this->backgroundColor, 0, 0, 100);
  }


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

}

void OgreAdaptor::Init(Display *display, 
                       XVisualInfo *visual, 
                       Window windowId, int width, int height)
{
  Ogre::NameValuePairList params;
  Ogre::StringVector paramsVector;

  this->display = display;
  this->visual = visual;
  this->windowId = windowId;

  // Setup the available resources 
  this->SetupResources(NULL);

  // Setup the rendering system, and don't create the context
  this->SetupRenderSystem(false);

  // Initialize the root node, and don't create a window
  this->window = this->root->initialise(false); 

  // Create the window
  this->CreateWindow(width,height);

  // Get the SceneManager, in this case a generic one
  this->sceneMgr = this->root->createSceneManager(Ogre::ST_GENERIC);

  // Set default mipmap level (NB some APIs ignore this)
  Ogre::TextureManager::getSingleton().setDefaultNumMipmaps( 2 );

  // Load Resources
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  // Default lighting
  this->sceneMgr->setAmbientLight(Ogre::ColourValue(0.8f, 0.8f, 0.8f, 1.0f)); 
  this->sceneMgr->setShadowTechnique( Ogre::SHADOWTYPE_STENCIL_ADDITIVE );

  OgreGLXWindowInterface* pWindowInterface = NULL;

  this->window->getCustomAttribute("GLXWINDOWINTERFACE", &pWindowInterface);

  //pWindowInterface->exposed(true);


  // Create our frame listener and register it
  this->frameListener = new OgreFrameListener();
  this->root->addFrameListener(this->frameListener);
}

void OgreAdaptor::SetupResources(XMLConfigNode *node)
{
  XMLConfigNode *sectionNode;
  XMLConfigNode *childNode;
  std::string path, sectionName, typeName, archName;

  if (!node)
  {
    gzthrow( "ogre resources xml nodemissing" );
  }

  path = node->GetString("path","",1);

  if (path.empty())
  {
    gzthrow( "empty resource path" );
  }

  sectionNode = node->GetChild();

  while(sectionNode)
  {
    sectionName = sectionNode->GetName();
    sectionName[0] = std::toupper(sectionName[0]);

    childNode = sectionNode->GetChild();

    while (childNode)
    {
      typeName = childNode->GetName();
      archName = path+"/"+childNode->GetValue();

      typeName[0] = std::toupper(typeName[0]);

      // Hack to make OGRE happy
      if (typeName == "Filesystem")
        typeName = "FileSystem";

      try
      {
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation( archName, typeName, sectionName); 
      }
      catch (Ogre::Exception)
      {
        gzthrow("Unable to load Ogre Resources.\nMake sure the resources path in the world file is set correctly.");
      }

      childNode = childNode->GetNext();
    }

    sectionNode = sectionNode->GetNext();
  }
}

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
    } while (selectedRenderSystem->getName().compare("OpenGL Rendering Subsystem")!= 0);

    if (selectedRenderSystem == NULL)
    {
      gzthrow( "unable to find rendering system" );
    }

    selectedRenderSystem->setConfigOption("Full Screen","No");
    selectedRenderSystem->setConfigOption("FSAA","2");

    if (create)
    {
      //selectedRenderSystem->setConfigOption("Video Mode","640 x 480 @ 16-bit colour");
      selectedRenderSystem->setConfigOption("Video Mode",this->videoMode);
    }

    this->root->setRenderSystem(selectedRenderSystem);
  }
}


void OgreAdaptor::CreateWindow(int width, int height)
{
  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;

  paramsVector.push_back( Ogre::StringConverter::toString( (int)(this->display) ) );
  paramsVector.push_back( Ogre::StringConverter::toString((int)this->visual->screen));

  paramsVector.push_back( Ogre::StringConverter::toString((int)this->windowId));
  paramsVector.push_back( Ogre::StringConverter::toString((int)(this->visual)));

  params["parentWindowHandle"] = Ogre::StringConverter::toString(paramsVector);

  this->window = this->root->createRenderWindow( "WindowName", width, height, false, &params);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Create a light source and attach it to the entity
void OgreAdaptor::CreateLight(XMLConfigNode *node, Entity *entity)
{
  Vector3 vec;
  double range,constant,linear,quad;

  // Create the light
  Ogre::Light *light(this->sceneMgr->createLight(entity->GetName()));

  // Set the light type
  std::string lightType = node->GetString("type","point",0);
  if (lightType == "point")
  {
    light->setType(Ogre::Light::LT_POINT);
  }
  else if (lightType == "directional")
  {
    light->setType(Ogre::Light::LT_DIRECTIONAL);
  }
  else if (lightType == "spot")
  {
    light->setType(Ogre::Light::LT_SPOTLIGHT);
  }

  // Set the diffuse color
  vec = node->GetVector3("diffuseColor",Vector3(1.0, 1.0, 1.0)); 
  light->setDiffuseColour(vec.x, vec.y, vec.z);

  // Sets the specular color
  vec = node->GetVector3("specularColor",Vector3(1.0, 1.0, 1.0)); 
  light->setSpecularColour(vec.x, vec.y, vec.z);

  // Set the direction which the light points
  vec = node->GetVector3("direction", Vector3(0.0, -1.0, 0.0));
  light->setDirection(vec.x, vec.y, vec.z);

  // Absolute range of light in world coordinates
  range = node->GetTupleDouble("attenuation",0,1000);

  // Constant factor. 1.0 means never attenuate, 0.0 is complete attenuation
  constant = node->GetTupleDouble("attenuation",1,1.0);

  // Linear factor. 1 means attenuate evenly over the distance
  linear = node->GetTupleDouble("attenuation",2,0);

  // Quadartic factor.adds a curvature to the attenuation formula
  quad = node->GetTupleDouble("attenuation",3,0);

  // Set attenuation
  light->setAttenuation(range, constant, linear, quad);

  // TODO: More options for Spot lights, etc.

  entity->GetSceneNode()->attachObject(light);
}
 

int OgreAdaptor::Render()
{
  Ogre::WindowEventUtilities::messagePump();

  OgreHUD::Instance()->Update();

  root->renderOneFrame();

  return 0;
}

// Load plugins
void OgreAdaptor::LoadPlugins(XMLConfigNode *node)
{
  std::string pathStr;
  std::string pluginStr;
  XMLConfigNode *pluginNode;

  if (!node)
  {
    gzthrow( "missing plugins xml node" );
  }

  // Get the path prefix
  pathStr = node->GetString("path","/usr/local/lib/OGRE",1);

  // Make sure a path has been specified
  if (pathStr.empty())
  {
    gzthrow( "no Plugin Path Set" );
  }

  // The first plugin
  pluginNode = node->GetChild();

  // Read all the plugins
  while (pluginNode)
  {
    pluginStr = pathStr + "/" + pluginNode->GetValue();
    gzmsg(5) << "OGRE: Load Plugin[" << pluginStr << "]\n";

    try
    {
      // Load the plugin into OGRE
      this->root->loadPlugin(pluginStr);
    }
    catch (Ogre::Exception e)
    {
      gzthrow("Unable to load Ogre Plugins.\nMake sure the plugins path in the world file is set correctly");
    }
    pluginNode = pluginNode->GetNext();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Use this function to set the pose of a scene node
void OgreAdaptor::SetSceneNodePose( Ogre::SceneNode *node, const Pose3d &pose )
{
  this->SetSceneNodePosition(node, pose.pos);
  this->SetSceneNodeRotation(node, pose.rot);
}

////////////////////////////////////////////////////////////////////////////////
/// Use this function to set the position of a scene node
void OgreAdaptor::SetSceneNodePosition( Ogre::SceneNode *node, const Vector3 &pos )
{
  node->setPosition(pos.y, pos.z, pos.x);
}

////////////////////////////////////////////////////////////////////////////////
/// Use this function to set the rotation of a scene node
void OgreAdaptor::SetSceneNodeRotation( Ogre::SceneNode *node, const Quatern &rot )
{
  node->setOrientation(rot.u, rot.y, rot.z, rot.x);
}

////////////////////////////////////////////////////////////////////////////////
// Helper function to create a camera
Ogre::Camera *OgreAdaptor::CreateCamera(const std::string &name, double nearClip, double farClip, Ogre::RenderTarget *renderTarget)
{
  Ogre::Camera *camera;
  Ogre::Viewport *cviewport;
 

  camera = this->sceneMgr->createCamera(name);

  camera->setNearClipDistance(nearClip);
  camera->setFarClipDistance(farClip);

  // Setup the viewport to use the texture
  cviewport = renderTarget->addViewport(camera);
  cviewport->setClearEveryFrame(true);
  cviewport->setBackgroundColour( *this->backgroundColor );
  cviewport->setOverlaysEnabled(false);
  
  camera->setAspectRatio(
      Ogre::Real(cviewport->getActualWidth()) / 
      Ogre::Real(cviewport->getActualHeight()) );
}
