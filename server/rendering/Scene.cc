#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneQuery.h>
#include <OGRE/OgreRoot.h>

#include "Global.hh"
#include "OgreCamera.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "UserCamera.hh"
#include "RTShaderSystem.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "Entity.hh"
#include "Grid.hh"
#include "OgreCamera.hh"

#include "Scene.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Scene::Scene(const std::string &name)
{
  this->name = name;
  this->manager = NULL;
  this->raySceneQuery = NULL;
  this->type = GENERIC;

  Grid *grid = new Grid(this, 1, 1, 10, Color(1,1,0,1));
  this->grids.push_back(grid);


  Param::Begin(&this->parameters);
  this->ambientP = new ParamT<Color>("ambient",Color(.1,.1,.1,.1),0);
  this->shadowsP = new ParamT<bool>("shadows", true, 0);
  this->skyMaterialP = new ParamT<std::string>("material","",1);
  this->backgroundColorP = new ParamT<Color>("backgroundColor",Color(0.5,0.5,0.5), 0);

  this->fogTypeP = new ParamT<std::string>("type","",0);
  this->fogColorP = new ParamT<Color>("color",Color(1,1,1,1),0);
  this->fogDensityP = new ParamT<double>("density",0,0);
  this->fogStartP = new ParamT<double>("linearStart",0,0);
  this->fogEndP = new ParamT<double>("linearEnd",1.0,0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Scene::~Scene()
{
  delete this->ambientP;
  delete this->shadowsP;
  delete this->backgroundColorP;
  delete this->skyMaterialP;

  delete this->fogTypeP;
  delete this->fogColorP;
  delete this->fogDensityP;
  delete this->fogStartP;
  delete this->fogEndP;

  for (unsigned int i=0; i < this->grids.size(); i++)
    delete this->grids[i];
  this->grids.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Load
void Scene::Load(XMLConfigNode *node)
{
  this->ambientP->Load(node);
  this->shadowsP->Load(node);
  this->backgroundColorP->Load(node);

  this->skyMaterialP->Load(node->GetChild("sky"));

  this->fogColorP->Load(node->GetChild("fog"));
  this->fogDensityP->Load(node->GetChild("fog"));
  this->fogTypeP->Load(node->GetChild("fog"));
  this->fogDensityP->Load(node->GetChild("fog"));
  this->fogStartP->Load(node->GetChild("fog"));
  this->fogEndP->Load(node->GetChild("fog"));

  // Get the SceneManager, in this case a generic one
  if (node && node->GetChild("bsp"))
    this->type = BSP;
  else
    this->type = GENERIC;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the scene
void Scene::Init(Ogre::Root *root)
{
  if (this->manager)
    root->destroySceneManager(this->manager);

  if (this->type == BSP)
    this->manager = root->createSceneManager("BspSceneManager");
  else
    this->manager = root->createSceneManager(Ogre::ST_GENERIC);

  for (unsigned int i=0; i < this->grids.size(); i++)
    this->grids[i]->Init();

  // Ambient lighting
  this->manager->setAmbientLight( (**this->ambientP).GetOgreColor() );

  // Create the sky
  if ( !(**this->skyMaterialP).empty() )
  {
    try
    {
      Ogre::Quaternion orientation;
      orientation.FromAngleAxis( Ogre::Degree(90), Ogre::Vector3(1,0,0));
      this->manager->setSkyDome(true, **(this->skyMaterialP),10,8, 4, true, orientation);
    }
    catch (int)
    {
      gzmsg(0) << "Unable to set sky dome to material[" 
               << **this->skyMaterialP << "]\n";
    }
  }

  // Create Fog
  this->SetFog(**this->fogTypeP, **this->fogColorP, **this->fogDensityP, 
               **this->fogStartP, **this->fogEndP);

  // Create ray scene query
  this->raySceneQuery = this->manager->createRayQuery( Ogre::Ray() );
  this->raySceneQuery->setSortByDistance(true);
  this->raySceneQuery->setQueryMask(Ogre::SceneManager::ENTITY_TYPE_MASK);

  // Register this scene the the real time shaders system
  RTShaderSystem::Instance()->AddScene(this);

  this->ApplyShadows();
}



////////////////////////////////////////////////////////////////////////////////
// Save the scene
void Scene::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << "  " << *(this->ambientP) << "\n";

  if ((**this->skyMaterialP).size())
  {
    stream << prefix << "  <sky>\n";
    stream << prefix << "    " << *(this->skyMaterialP) << "\n";
    stream << prefix << "  </sky>\n";
  }

  stream << prefix << "  <fog>\n";
  stream << prefix << "    " << *this->fogTypeP << "\n";
  stream << prefix << "    " << *this->fogColorP << "\n";
  stream << prefix << "    " << *this->fogStartP << "\n";
  stream << prefix << "    " << *this->fogEndP << "\n";
  stream << prefix << "    " << *this->fogDensityP << "\n";
  stream << prefix << "  </fog>\n";

  stream << prefix << "  " << *(this->shadowsP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Get the OGRE scene manager
Ogre::SceneManager *Scene::GetManager() const
{
  return this->manager;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of the scene
std::string Scene::GetName() const
{
  return this->name;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the ambient color
void Scene::SetAmbientColor(const Color &color)
{
  this->ambientP->SetValue(color);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ambient color
Color Scene::GetAmbientColor() const
{
  return **this->ambientP;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the background color
void Scene::SetBackgroundColor(const Color &color)
{
  this->backgroundColorP->SetValue(color);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the background color
Color Scene::GetBackgroundColor() const
{
  return **this->backgroundColorP;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the scene type
void Scene::SetType(Scene::SceneType type)
{
  this->type = type;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the scene type
Scene::SceneType Scene::GetType() const
{
  return this->type;
}

////////////////////////////////////////////////////////////////////////////////
/// Create a grid
void Scene::CreateGrid(uint32_t cell_count, float cell_length, 
                       float line_width, const Color &color )
{
  Grid *grid = new Grid(this, cell_count, cell_length, line_width, color);

  if (this->manager)
    grid->Init();

  this->grids.push_back(grid);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the grid
Grid *Scene::GetGrid(unsigned int index)
{
  if (index >= this->grids.size())
  {
    std::cerr << "Scene::GetGrid() Invalid index\n";
    return NULL;
  }

  return this->grids[index];
}



////////////////////////////////////////////////////////////////////////////////
// Update the user cameras
void Scene::UpdateCameras()
{
  UserCamera *userCam;

  std::vector<OgreCamera*>::iterator iter;

  // Draw all the non-user cameras
  for (iter = this->cameras.begin(); iter != this->cameras.end(); iter++)
    (*iter)->Render();

  for (iter = this->cameras.begin(); iter != this->cameras.end(); iter++)
    (*iter)->PostRender();

  Ogre::HardwarePixelBufferSharedPtr pixelBuffer;
  Ogre::RenderTexture *rTexture;
  Ogre::Viewport* renderViewport;

  // Get access to the buffer and make an image and write it to file
  if (false && this->manager && !this->manager->getShadowTexture(0).isNull())
  {
    pixelBuffer = this->manager->getShadowTexture(0)->getBuffer();
    rTexture = pixelBuffer->getRenderTarget();

    Ogre::PixelFormat format = pixelBuffer->getFormat();
    renderViewport = rTexture->getViewport(0);

    int imgsize = 512;
    size_t size = Ogre::PixelUtil::getMemorySize(imgsize, imgsize, 1, format);

    //// Allocate buffer
    float *saveFrameBuffer = new float[imgsize*imgsize*3];

    memset(saveFrameBuffer, 0, size);

    Ogre::PixelBox box(imgsize, imgsize, 1, format, saveFrameBuffer);

    //std::cout << "Depth[" << this->manager->getShadowTexture(0)->getDepth() << "]\n";
    pixelBuffer->blitToMemory( box );

    //std::cout << "SIZE[" << size << "] [" << imgsize * imgsize * (4*3) << "]\n";

    float max = 0.0;
    float min = 1000.0;
    double sum = 0.0;
    FILE *file = fopen("/tmp/nate.data","w");
    for (unsigned int y = 0; y < imgsize; y+=4)
    {
      for (unsigned int x=0;x<imgsize;x+=4)
      {
        float d = saveFrameBuffer[ y*(imgsize*3) + x*3 + 0];
        float dd = saveFrameBuffer[ y*(imgsize*3)+ x*3 + 1];
        sum += d;
        if (d < min)
          min = d;
        if (d > max)
          max = d;

        fprintf(file, "%d %d %f\n",x,y,d);
      }
      fprintf(file,"\n\n");
    }

    printf("Min[%f] Max[%f] Avg[%f]\n", min, max, sum / (imgsize*imgsize));

    fclose(file);

    //exit(1);
    delete [] saveFrameBuffer;
  }

  //this->manager->_handleLodEvents();
}

////////////////////////////////////////////////////////////////////////////////
/// Get an entity at a pixel location using a camera. Used for mouse picking. 
Entity *Scene::GetEntityAt(OgreCamera *camera, Vector2<int> mousePos, std::string &mod) 
{
  Entity *entity = NULL;
  Ogre::Camera *ogreCam = camera->GetOgreCamera();
  Ogre::Vector3 camPos = ogreCam->getPosition();

  Ogre::Real closest_distance = -1.0f;
  Ogre::Ray mouseRay = ogreCam->getCameraToViewportRay(
      (float)mousePos.x / ogreCam->getViewport()->getActualWidth(), 
      (float)mousePos.y / ogreCam->getViewport()->getActualHeight() );

  this->raySceneQuery->setRay( mouseRay );

  // Perform the scene query
  Ogre::RaySceneQueryResult &result = this->raySceneQuery->execute();
  Ogre::RaySceneQueryResult::iterator iter = result.begin();
  Ogre::Entity *closestEntity = NULL;

  for (iter = result.begin(); iter != result.end(); iter++)
  {
    // is the result a MovableObject
    if (iter->movable && iter->movable->getMovableType().compare("Entity") == 0)
    {
      Ogre::Entity *pentity = static_cast<Ogre::Entity*>(iter->movable);

      // mesh data to retrieve         
      size_t vertex_count;
      size_t index_count;
      Ogre::Vector3 *vertices;
      unsigned long *indices;

      // Get the mesh information
      OgreCreator::GetMeshInformation(pentity->getMesh(), vertex_count, 
          vertices, index_count, indices,             
          pentity->getParentNode()->_getDerivedPosition(),
          pentity->getParentNode()->_getDerivedOrientation(),
          pentity->getParentNode()->_getDerivedScale());

      bool new_closest_found = false;
      for (int i = 0; i < static_cast<int>(index_count); i += 3)
      {
        // check for a hit against this triangle
        std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(mouseRay, vertices[indices[i]], vertices[indices[i+1]], vertices[indices[i+2]], true, false);

        // if it was a hit check if its the closest
        if (hit.first)
        {
          if ((closest_distance < 0.0f) || (hit.second < closest_distance))
          {
            // this is the closest so far, save it off
            closest_distance = hit.second; 
            new_closest_found = true;
          }
        }
      }

      delete [] vertices;
      delete [] indices;

      if (new_closest_found)
      {
        closestEntity = pentity;
        break;
      }
    }
  }

  mod = "";
  if (closestEntity)
  {
    if (closestEntity->getUserAny().getType() == typeid(std::string))
      mod = Ogre::any_cast<std::string>(closestEntity->getUserAny());

    OgreVisual* const* vis = Ogre::any_cast<OgreVisual*>(&closestEntity->getUserAny());

    if (vis && (*vis)->GetOwner())
    {
      entity = (*vis)->GetOwner();
      return entity;
    }
  }

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the world pos of a the first contact at a pixel location
Vector3 Scene::GetFirstContact(OgreCamera *camera, Vector2<int> mousePos)
{
  Ogre::Camera *ogreCam = camera->GetOgreCamera();
  Ogre::Real closest_distance = -1.0f;
  Ogre::Ray mouseRay = ogreCam->getCameraToViewportRay(
      (float)mousePos.x / ogreCam->getViewport()->getActualWidth(), 
      (float)mousePos.y / ogreCam->getViewport()->getActualHeight() );

  this->raySceneQuery->setRay( mouseRay );

  // Perform the scene query
  Ogre::RaySceneQueryResult &result = this->raySceneQuery->execute();
  Ogre::RaySceneQueryResult::iterator iter = result.begin();

  Ogre::Vector3 pt = mouseRay.getPoint(iter->distance);

  return Vector3(pt.x, pt.y, pt.z);
}

////////////////////////////////////////////////////////////////////////////////
// Register a camera
void Scene::RegisterCamera( OgreCamera *cam )
{
  this->cameras.push_back(cam);
}

////////////////////////////////////////////////////////////////////////////////
/// Unregister a user camera
void Scene::UnregisterCamera( OgreCamera *cam )
{
  std::vector<OgreCamera*>::iterator iter;
  for(iter=this->cameras.begin();iter != this->cameras.end();iter++)
  {
    if ((*iter) == cam)
    {
      this->cameras.erase(iter);
      break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Print scene graph
void Scene::PrintSceneGraph()
{
  this->PrintSceneGraphHelper("", this->manager->getRootSceneNode());
}

////////////////////////////////////////////////////////////////////////////////
/// Print scene graph
void Scene::PrintSceneGraphHelper(std::string prefix, Ogre::Node *node)
{
  std::cout << prefix << node->getName() << std::endl;

  prefix += "  ";
  for (unsigned int i=0; i < node->numChildren(); i++)
  {
    this->PrintSceneGraphHelper( prefix, node->getChild(i) );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Draw a named line
void Scene::DrawLine(const Vector3 &start, const Vector3 &end, 
                     const std::string &name)
{
  Ogre::SceneNode *node = NULL;
  Ogre::ManualObject *obj = NULL;
  bool attached = false;

  if ( this->manager->hasManualObject(name))
  {
    node = this->manager->getSceneNode(name);
    obj = this->manager->getManualObject(name);
    attached = true;
  }
  else
  {
    node = this->manager->getRootSceneNode()->createChildSceneNode(name);
    obj = this->manager->createManualObject(name); 
  }

  node->setVisible(true);
  obj->setVisible(true);

  obj->clear();
  obj->begin("Gazebo/Red", Ogre::RenderOperation::OT_LINE_LIST); 
  obj->position(start.x, start.y, start.z); 
  obj->position(end.x, end.y, end.z); 
  obj->end(); 

  if (!attached)
    node->attachObject(obj);
}

////////////////////////////////////////////////////////////////////////////////
// Set fog for this scene
void Scene::SetFog( std::string type, const Color &color, double density, 
                    double start, double end )
{
  Ogre::FogMode fogType = Ogre::FOG_NONE;

  if (type == "linear")
    fogType = Ogre::FOG_LINEAR;
  else if (type == "exp")
    fogType = Ogre::FOG_EXP;
  else if (type == "exp2")
    fogType = Ogre::FOG_EXP2;
  else
    type = "none";

  this->manager->setFog( fogType, color.GetOgreColor(), density, start, end );

  this->fogTypeP->SetValue(type);
  this->fogColorP->SetValue(color);
  this->fogDensityP->SetValue(density);
  this->fogStartP->SetValue(start);
  this->fogEndP->SetValue(end);
}

////////////////////////////////////////////////////////////////////////////////
/// Hide a visual
void Scene::SetVisible(const std::string &name, bool visible)
{
  if (this->manager->hasSceneNode(name))
    this->manager->getSceneNode(name)->setVisible(visible);

  if ( this->manager->hasManualObject(name))
    this->manager->getManualObject(name)->setVisible(visible);
}

////////////////////////////////////////////////////////////////////////////////
void Scene::ApplyShadows()
{
  // Allow a total of 4 shadow casters per scene
  const int numShadowTextures = 3;

  /*this->manager->setShadowFarDistance(500);
  this->manager->setShadowTextureCount(numShadowTextures);

  this->manager->setShadowTextureSize(1024);
  this->manager->setShadowTexturePixelFormat(Ogre::PF_FLOAT32_RGB);
  this->manager->setShadowTextureSelfShadow(false);
  this->manager->setShadowCasterRenderBackFaces(true);
  this->manager->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE_INTEGRATED);
  this->manager->setShadowTextureCasterMaterial("shadow_caster");

  const unsigned numShadowRTTs = this->manager->getShadowTextureCount();
  for (unsigned i = 0; i < numShadowRTTs; ++i) 
  {
    Ogre::TexturePtr tex = this->manager->getShadowTexture(i);
    Ogre::Viewport *vp = tex->getBuffer()->getRenderTarget()->getViewport(0);
    vp->setBackgroundColour(Ogre::ColourValue(0, 0, 0, 1));
    vp->setClearEveryFrame(true);

    //Ogre::CompositorManager::getSingleton().addCompositor(vp, "blur");
    //Ogre::CompositorManager::getSingleton().setCompositorEnabled(vp, "blur", true);
  }
  */

  this->manager->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE_INTEGRATED);
  Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(Ogre::TFO_ANISOTROPIC);

  this->manager->setShadowFarDistance(100);
  this->manager->setShadowTextureCountPerLightType(Ogre::Light::LT_DIRECTIONAL, 
                                                   numShadowTextures);

  this->manager->setShadowTextureCount(numShadowTextures+1);

  this->manager->setShadowTextureCasterMaterial("shadow_caster");
  this->manager->setShadowCasterRenderBackFaces(true);
  this->manager->setShadowTextureSelfShadow(false);

  // PSSM Stuff
  this->manager->setShadowTextureConfig(0, 512, 512, Ogre::PF_FLOAT32_RGB);
  this->manager->setShadowTextureConfig(1, 512, 512, Ogre::PF_FLOAT32_RGB);
  this->manager->setShadowTextureConfig(2, 512, 512, Ogre::PF_FLOAT32_RGB);
  this->manager->setShadowTextureConfig(3, 512, 512, Ogre::PF_FLOAT32_RGB);


  // DEBUG CODE: Will display three overlay panels that show the contents of 
  // the shadow maps
  // add the overlay elements to show the shadow maps:
  // init overlay elements
  /*Ogre::OverlayManager& mgr = Ogre::OverlayManager::getSingleton();
  Ogre::Overlay* overlay = mgr.create("DebugOverlay");
   for (size_t i = 0; i < 4; ++i) {
    Ogre::TexturePtr tex = this->manager->getShadowTexture(i);

    // Set up a debug panel to display the shadow
    Ogre::MaterialPtr debugMat =Ogre:: MaterialManager::getSingleton().create(
        "Ogre/DebugTexture" + Ogre::StringConverter::toString(i),
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    debugMat->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    Ogre::TextureUnitState *t = debugMat->getTechnique(0)->getPass(0)->createTextureUnitState(tex->getName());
    t->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

    Ogre::OverlayContainer* debugPanel = (Ogre::OverlayContainer*)
      (Ogre::OverlayManager::getSingleton().createOverlayElement("Panel", "Ogre/DebugTexPanel" + Ogre::StringConverter::toString(i)));
    debugPanel->_setPosition(0.8, i*0.25);
    debugPanel->_setDimensions(0.2, 0.24);
    debugPanel->setMaterialName(debugMat->getName());
    overlay->add2D(debugPanel);
    overlay->show();
  }
   */
 
}
