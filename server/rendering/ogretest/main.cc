#include <Ogre.h>
#include <string>

#include <sys/types.h>
#include <dirent.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>

#include "GLWindow.hh"

Ogre::Root *root;
Ogre::SceneManager *sceneMgr;
Ogre::RenderWindow *window;
Ogre::Camera *camera;
Ogre::Viewport *viewport;

Display *myDisplay;
long windowId;

bool noGui;

GLWindow *gui;

void CreateWindow(Display *display, int screen, long winId, unsigned int width, 
                  unsigned int height)
{
  windowId = winId;
  myDisplay = display;
  std::stringstream ogreHandle;

  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;

/*  std::string screenStr = DisplayString((long)display);
  std::string::size_type dotPos = screenStr.find(".");
  screenStr = screenStr.substr(dotPos+1, screenStr.size());

  int attrList[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, 16, 
                    GLX_STENCIL_SIZE, 8, None };
  XVisualInfo *vi = glXChooseVisual(display, DefaultScreen((long)display), 
                                    attrList);
  */

  /*ogreHandle << (unsigned long)display 
             << ":" << screenStr 
             << ":" << (unsigned long)winId 
             << ":" << (unsigned long)vi;
             */

  XSync(display, false);

  ogreHandle << winId;
  params["parentWindowHandle"] = ogreHandle.str();
  //params["currentGLContext"] = "true";

  //params["vsync"] = "true";

  window = root->createRenderWindow( "Window", width, height, false, &params);
  window->setActive(true);
  window->setVisible(true);
  window->setAutoUpdated(true);
}

void ResizeWindow(int w, int h)
{
  //XResizeWindow(myDisplay, windowId, w, h);
  window->resize(w,h);
  window->windowMovedOrResized();
  viewport->setDimensions(0,0,1,1);
}

void SetupResources(void)
{
  Ogre::ConfigFile cf;
  cf.load("resources.cfg");

  Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

  Ogre::String secName, typeName, archName;
  while (seci.hasMoreElements())
  {
    secName = seci.peekNextKey();
    Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
    Ogre::ConfigFile::SettingsMultiMap::iterator i;
    for (i = settings->begin(); i != settings->end(); ++i)
    {
      typeName = i->first;
      archName = i->second;
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          archName, typeName, secName);
    }
  }
}


void CreateScene()
{
  Ogre::Light *sunlight, *pointlight;

  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
  sceneMgr->setAmbientLight( Ogre::ColourValue(0,0,0) );

  sunlight = sceneMgr->createLight("Sunlight");
  sunlight->setType(Ogre::Light::LT_SPOTLIGHT);
  sunlight->setPosition(1500, 1750, 1300);
  sunlight->setSpotlightRange( Ogre::Degree(30), Ogre::Degree(50) );
  Ogre::Vector3 dir;
  dir = -sunlight->getPosition();
  dir.normalise();
  sunlight->setDirection(dir);
  sunlight->setDiffuseColour(0.35, 0.35, 0.38);
  sunlight->setSpecularColour(0.9, 0.9, 1);

  pointlight = sceneMgr->createLight("pointlight");
  pointlight->setDiffuseColour( Ogre::ColourValue(0.2, 0.1, 0.0) );
  pointlight->setSpecularColour(1,1,1);
  pointlight->setAttenuation(8000,1,0.0005,0);
  pointlight->setPosition(10, 50, 0);

  Ogre::SceneNode *node;
  node = sceneMgr->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity *athene = sceneMgr->createEntity("athene", "athene.mesh");
  athene->setMaterialName("Examples/Athene/NormalMapped");
  node->attachObject( athene );
  node->translate(0, -27, 0);
  node->yaw(Ogre::Degree(90));

  Ogre::MovablePlane *plane = new Ogre::MovablePlane("mplane");
  plane->normal = Ogre::Vector3::UNIT_Y;
  plane->d = 107;
  Ogre::MeshManager::getSingleton().createPlane("Myplane",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, *plane,
    1500,1500,50,50,true,1,5,5, Ogre::Vector3::UNIT_Z);

  Ogre::Entity *planeEnt = sceneMgr->createEntity( "plane", "Myplane" );
  planeEnt->setMaterialName("Gazebo/Rockwall");
  planeEnt->setCastShadows(false);
  sceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(planeEnt);

}

int main(int argc, char **argv)
{
  noGui = true;

  if (argc > 1)
    if (strcmp(argv[1], "gui") == 0 )
      noGui = false;

  // Pipe ogre output to a log file
  Ogre::LogManager *logManager = new Ogre::LogManager();
  logManager->createLog("Ogre.log", true, false, false);

  root = new Ogre::Root("plugins.cfg", "ogre.cfg");

  SetupResources();
  root->showConfigDialog();

  window = root->initialise(noGui);

  if (!noGui)
  {
    printf("Creating fltk gui\n");
    gui = new GLWindow(0,0,800,600,"Ogre TEST");
    gui->Init();
  }

  // Create the scene manager
  sceneMgr = root->createSceneManager(Ogre::ST_GENERIC);

  // Load Resources
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  // Create the camera
  camera = sceneMgr->createCamera("PlayerCam");
  camera->setPosition( Ogre::Vector3(0, 0, 500));
  camera->lookAt( Ogre::Vector3(0, 0, -300) );
  camera->setNearClipDistance(5);

  // Create viewports
  viewport = window->addViewport(camera);
  viewport->setBackgroundColour( Ogre::ColourValue(0,0,0) );
  camera->setAspectRatio( Ogre::Real(viewport->getActualWidth()) /
                          Ogre::Real(viewport->getActualHeight()) );

  // Create scene
  CreateScene();

  while (true)
  {
    root->_fireFrameStarted();
    window->update();
    gui->Update();
    root->_fireFrameEnded();
  }

  root->startRendering();

  delete root;
  return 0;
}

