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
  Ogre::Light *pointlight, *mSunLight;

  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
  //sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_MODULATIVE);
  //sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_MODULATIVE);
  //sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);

  //sceneMgr->setShadowTextureCasterMaterial("Ogre/DepthShadowmap/Caster/Float");
  //sceneMgr->setShadowTextureReceiverMaterial("Ogre/DepthShadowmap/Receiver/Float");
      
  sceneMgr->setAmbientLight( Ogre::ColourValue(0.0,0.0,0.0) );


  mSunLight = sceneMgr->createLight("SunLight");
  mSunLight->setType(Ogre::Light::LT_SPOTLIGHT);
  mSunLight->setPosition(1500,1750,1300);
  mSunLight->setSpotlightRange(Ogre::Degree(30), Ogre::Degree(50));
  Ogre::Vector3 dir;
  dir = -mSunLight->getPosition();
  dir.normalise();
  mSunLight->setDirection(dir);
  mSunLight->setDiffuseColour(0.35, 0.35, 0.38);
  mSunLight->setSpecularColour(0.9, 0.9, 1);
  mSunLight->setCastShadows(true);


  pointlight = sceneMgr->createLight("pointlight");
  pointlight->setDiffuseColour( Ogre::ColourValue(0.8, 0.8, 0.8) );
  pointlight->setSpecularColour(1,1,1);
  pointlight->setAttenuation(8000, .1, 0.0005,0);
  pointlight->setPosition(100, 500, 0);
  pointlight->setCastShadows(true);


  Ogre::MovablePlane *plane = new Ogre::MovablePlane("mplane");
  plane->normal = Ogre::Vector3::UNIT_Y;
  plane->d = 107;
  Ogre::MeshManager::getSingleton().createPlane("Myplane",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, *plane,
    1500,1500,50,50,true,1,5,5, Ogre::Vector3::UNIT_Z);

  Ogre::Entity *planeEnt = sceneMgr->createEntity( "plane", "Myplane" );
  //planeEnt->setMaterialName("Ogre/DepthShadowmap/Receiver/RockWall");
  planeEnt->setCastShadows(false);
  sceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(planeEnt);


  Ogre::SceneNode *node = sceneMgr->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity *pEnt = sceneMgr->createEntity( "COl", "column.mesh" );
  pEnt->setMaterialName("Gazebo/Grey");
  node->attachObject( pEnt );
  node->translate(0,0, 0);

  /*sceneMgr->setShadowTextureSettings(512,2);
  sceneMgr->setShadowColour(Ogre::ColourValue(0.5, 0.5, 0.5));

  sceneMgr->setShadowTexturePixelFormat(Ogre::PF_FLOAT16_R);
  sceneMgr->setShadowTextureCasterMaterial("Ogre/DepthShadowmap/Caster/Float");
  sceneMgr->setShadowTextureReceiverMaterial("Ogre/DepthShadowmap/Receiver/Float");
  */
 
  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_MODULATIVE);
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

    // Create scene
  CreateScene();

// Create the camera
  camera = sceneMgr->createCamera("Camera");
  camera->setPosition( Ogre::Vector3(350, 200, 800));
  camera->lookAt( Ogre::Vector3(0,10 ,0) );
  camera->setNearClipDistance(5);
  camera->setFarClipDistance(100000);

  // Create viewports
  viewport = window->addViewport(camera);
  viewport->setBackgroundColour( Ogre::ColourValue(0,0,0) );
  camera->setAspectRatio( Ogre::Real(viewport->getActualWidth()) /
                          Ogre::Real(viewport->getActualHeight()) );


  root->startRendering();

  delete root;
  return 0;
}
