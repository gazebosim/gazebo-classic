#include <Ogre.h>

Ogre::Root *root;
Ogre::SceneManager *sceneMgr;
Ogre::RenderWindow *window;
Ogre::Camera *camera;
Ogre::Viewport *viewport;

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
  Ogre::Light *pointlight;

  sceneMgr->setAmbientLight( Ogre::ColourValue(0,0,0) );

  pointlight = sceneMgr->createLight("pointlight");
  pointlight->setDiffuseColour( Ogre::ColourValue(0.2, 0.2, 0.2) );
  pointlight->setSpecularColour(1,1,1);
  pointlight->setAttenuation(8000, .1, 0.001,0);
  pointlight->setPosition(0, 50, 0);

  Ogre::MovablePlane *plane = new Ogre::MovablePlane("mplane");
  plane->normal = Ogre::Vector3::UNIT_Y;
  plane->d = 100;
  Ogre::MeshManager::getSingleton().createPlane("Myplane",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, *plane,
    5500,5500,20,20,true,1,5,5, Ogre::Vector3::UNIT_Z);

  Ogre::Entity *planeEnt = sceneMgr->createEntity( "plane", "Myplane" );
  planeEnt->setMaterialName("test");
  sceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(planeEnt);
}

int main(int argc, char **argv)
{

  // Pipe ogre output to a log file
  Ogre::LogManager *logManager = new Ogre::LogManager();
  logManager->createLog("Ogre.log", true, false, false);

  root = new Ogre::Root("plugins.cfg", "ogre.cfg");

  SetupResources();
  root->showConfigDialog();

  window = root->initialise(true);

  // Create the scene manager
  sceneMgr = root->createSceneManager(Ogre::ST_GENERIC);

  // Load Resources
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  // Create the camera
  camera = sceneMgr->createCamera("Camera");
  camera->setPosition( Ogre::Vector3(0, 1500, 0));
  camera->lookAt( Ogre::Vector3(0,0 , -150) );
  camera->setNearClipDistance(5);

  // Create viewports
  viewport = window->addViewport(camera);
  viewport->setBackgroundColour( Ogre::ColourValue(0,0,0) );
  camera->setAspectRatio( Ogre::Real(viewport->getActualWidth()) /
                          Ogre::Real(viewport->getActualHeight()) );

  // Create scene
  CreateScene();

  root->startRendering();

  delete root;
  return 0;
}
