#include <CEGUISystem.h>
#include <CEGUISchemeManager.h>
#include <OgreCEGUIRenderer.h>
#include <OgreLogManager.h>
#include <Ogre.h>
#include "OgreFrameListener.hh"
#include "OgreAdaptor.hh"


OgreAdaptor *OgreAdaptor::myself = NULL;

/// Constructor
OgreAdaptor::OgreAdaptor()
{

  printf("Create Log Manager\n");

  // Create a new log manager and prevent output from going to stdout
  this->logManager = new Ogre::LogManager();
  this->logManager->createLog("Ogre.log", true, false, false);

  printf("Create Root\n");
  // Make the root 
  this->root = new Ogre::Root(); 

  printf("Post Create Root\n");
}

/// Destructor
OgreAdaptor::~OgreAdaptor()
{
}

OgreAdaptor *OgreAdaptor::Instance()
{
  printf("Before\n");
  if (myself == NULL)
    myself = new OgreAdaptor();
  printf("After\n");

  return myself;
}


int OgreAdaptor::Init()
{
  printf("init1\n");
  // Setup the available resources 
  this->SetupResources();

  printf("init2\n");
  // Setup the rendering system, and create the context
  this->SetupRenderSystem(true);

  printf("init3\n");
  // Initialize the root node, and create a window
  this->window = this->root->initialise(true); 

  printf("init4\n");
  // Get the SceneManager, in this case a generic one
  this->sceneMgr = this->root->createSceneManager(Ogre::ST_GENERIC);

  printf("init5\n");
  this->CreateCameras();

  printf("init6\n");
  this->CreateViewports();

  printf("init7\n");
  // Set default mipmap level (NB some APIs ignore this)
  Ogre::TextureManager::getSingleton().setDefaultNumMipmaps( 5 );

  printf("init8\n");
  // Load Resources
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  printf("init9\n");
  // Default lighting
  this->sceneMgr->setAmbientLight(Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f)); 
  this->sceneMgr->setShadowTechnique( Ogre::SHADOWTYPE_STENCIL_ADDITIVE );

  printf("init10\n");
  // Add a sky dome to our scene
  this->sceneMgr->setSkyDome(true,"Gazebo/CloudySky",5,8);

  printf("init11\n");
  // Create our frame listener and register it
  this->frameListener = new OgreFrameListener(this);
  this->root->addFrameListener(this->frameListener);

  printf("K\n");
  // CEGUI setup
  this->guiRenderer = new CEGUI::OgreCEGUIRenderer(this->window, 
      Ogre::RENDER_QUEUE_OVERLAY, false, 0, this->sceneMgr);

  /*
  printf("L\n");
  this->guiSystem = new CEGUI::System(this->guiRenderer);

  // Mouse
  CEGUI::SchemeManager::getSingleton().loadScheme((CEGUI::utf8*)"TaharezLook.scheme");
  CEGUI::MouseCursor::getSingleton().setImage("TaharezLook", "MouseArrow");
  */

  return 0;
}

int OgreAdaptor::Init(Display *display, XVisualInfo *visual, 
    Window windowId, int width, int height)
{
  Ogre::NameValuePairList params;
  Ogre::StringVector paramsVector;

  this->display = display;
  this->visual = visual;
  this->windowId = windowId;

  // Setup the available resources 
  this->SetupResources();

  // Setup the rendering system, and don't create the context
  this->SetupRenderSystem(false);

  // Initialize the root node, and don't create a window
  this->window = this->root->initialise(false); 

  // Create the window
  this->CreateWindow(width,height);

  // Get the SceneManager, in this case a generic one
  this->sceneMgr = this->root->createSceneManager(Ogre::ST_GENERIC);

  this->CreateCameras();

  this->CreateViewports();

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
  this->frameListener = new OgreFrameListener(this);
  this->root->addFrameListener(this->frameListener);


  return 0;
}

void OgreAdaptor::SetupResources()
{
  // Load resource paths from config file
  Ogre::ConfigFile cf;
  cf.load( "resources.cfg");

  // Go through all sections & settings in the file
  Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

  Ogre::String secName, typeName, archName;
  while ( seci.hasMoreElements())
  {
    secName = seci.peekNextKey(); 
    Ogre::ConfigFile::SettingsMultiMap* settings = seci.getNext();
    Ogre::ConfigFile::SettingsMultiMap::iterator i;
    for ( i = settings->begin(); i != settings->end(); ++i)
    {
      typeName = i->first;
      archName = i->second;
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation( archName, typeName, secName); 
    }
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
      printf("ERROR!!!\n");

    selectedRenderSystem->setConfigOption("Full Screen","No");
    selectedRenderSystem->setConfigOption("FSAA","2");

    if (create)
    {
      selectedRenderSystem->setConfigOption("Video Mode","800 x 600 @ 16-bit colour");
    }

    this->root->setRenderSystem(selectedRenderSystem);
  }
}

void OgreAdaptor::CreateCameras()
{
  // Create the camera
  /*this->camera = this->sceneMgr->createCamera( "PlayerCam" );
  this->camera->setNearClipDistance(2);

  this->cameraNode = this->sceneMgr->getRootSceneNode()->createChildSceneNode("CameraNode", Ogre::Vector3(0,5,5));
  this->cameraNode->yaw( Ogre::Degree(45) );
  this->cameraPitchNode = this->cameraNode->createChildSceneNode("CameraPitchNode");
  this->cameraPitchNode->attachObject(this->camera);
  */
}

void OgreAdaptor::CreateViewports()
{
  // Create one viewport, entire window
  /*Ogre::Viewport* vp = this->window->addViewport(this->camera);
  vp->setBackgroundColour( Ogre::ColourValue(0,0,0) );

  // Alter the camera aspect ratio to match the viewport
  this->camera->setAspectRatio( Ogre::Real(vp->getActualWidth()) / Ogre::Real(vp->getActualHeight()));
  */
}

void OgreAdaptor::CreateScene()
{
  // Create the scene
  Ogre::Entity *ent = this->sceneMgr->createEntity( "Ninja", "ninja.mesh" );
  ent->setCastShadows(true);
  this->sceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(ent);

  Ogre::Plane plane(Ogre::Vector3::UNIT_Y,0);
  Ogre::MeshManager::getSingleton().createPlane("ground",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,plane,
      1500,1500,20,20,true,1,5,5,Ogre::Vector3::UNIT_Z);

  ent = this->sceneMgr->createEntity( "GroundEntity", "ground");
  this->sceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(ent);
  ent->setMaterialName("Examples/Rockwall");
  ent->setCastShadows(false);

  Ogre::Light *light = this->sceneMgr->createLight( "Light1" );
  light->setType( Ogre::Light::LT_POINT );
  light->setPosition( Ogre::Vector3(0,200,-50) );
  light->setDiffuseColour( 1.0, 0.0, 0.0 );
  light->setSpecularColour( 1.0, 0.0, 0.0 );
}

void OgreAdaptor::CreateWindow(int width, int height)
{
  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;

  paramsVector.push_back( Ogre::StringConverter::toString( (int)this->display ) );
  paramsVector.push_back( Ogre::StringConverter::toString((int)this->visual->screen));

  paramsVector.push_back( Ogre::StringConverter::toString((int)this->windowId));
  paramsVector.push_back( Ogre::StringConverter::toString((int)this->visual));

  params["parentWindowHandle"] = Ogre::StringConverter::toString(paramsVector);

  this->window = this->root->createRenderWindow( "WindowName", width, height, false, &params);
}

int OgreAdaptor::Render()
{
  Ogre::PlatformManager::getSingleton().messagePump(window);

  root->renderOneFrame();

  return 0;
}
