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
#include <boost/lexical_cast.hpp>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneQuery.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/Ogre.h>

#include "Light.hh"
#include "Global.hh"
#include "Visual.hh"
#include "OgreAdaptor.hh"
#include "UserCamera.hh"
#include "Camera.hh"
//#include "RTShaderSystem.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "Entity.hh"
#include "Grid.hh"

#include "Scene.hh"

using namespace gazebo;

unsigned int Scene::idCounter = 0;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Scene::Scene(const std::string &name)
{
  this->id = idCounter++;
  this->idString = boost::lexical_cast<std::string>(this->id);

  this->name = name;
  this->manager = NULL;
  this->raySceneQuery = NULL;
  this->type = GENERIC;

  Grid *grid = new Grid(this, 1, 1, 10, Color(1,1,0,1));
  this->grids.push_back(grid);


  Param::Begin(&this->parameters);
  this->ambientP = new ParamT<Color>("ambient",Color(.1,.1,.1,.1),0);
  this->shadowsP = new ParamT<bool>("shadows", false, 0);
  this->skyMaterialP = new ParamT<std::string>("material","",1);
  this->backgroundColorP = new ParamT<Color>("background_color",Color(0.5,0.5,0.5), 0);

  this->fogTypeP = new ParamT<std::string>("type","",0);
  this->fogColorP = new ParamT<Color>("color",Color(1,1,1,1),0);
  this->fogDensityP = new ParamT<double>("density",0,0);
  this->fogStartP = new ParamT<double>("linear_start",0,0);
  this->fogEndP = new ParamT<double>("linear_end",1.0,0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Scene::~Scene()
{
  VisualMap::iterator iter;
  for (iter = this->visuals.begin(); iter != this->visuals.end(); iter++)
    delete iter->second;
  this->visuals.clear();

  LightMap::iterator lightIter;
  for (lightIter = this->lights.begin(); lightIter != this->lights.end(); lightIter++)
    delete lightIter->second;
  this->lights.clear();

  // Remove a scene
  //RTShaderSystem::Instance()->RemoveScene( this );

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

  for (unsigned int i=0; i < this->cameras.size(); i++)
    delete this->cameras[i];
  this->cameras.clear();

  for (unsigned int i=0; i < this->userCameras.size(); i++)
    delete this->userCameras[i];
  this->userCameras.clear();
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
void Scene::Init()
{
  Ogre::Root *root = OgreAdaptor::Instance()->root;

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
  //RTShaderSystem::Instance()->AddScene(this);
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
Grid *Scene::GetGrid(unsigned int index) const
{
  if (index >= this->grids.size())
  {
    std::cerr << "Scene::GetGrid() Invalid index\n";
    return NULL;
  }

  return this->grids[index];
}

////////////////////////////////////////////////////////////////////////////////
//Create a camera
Camera *Scene::CreateCamera(const std::string &name)
{
  Camera *camera = new Camera(this->name + "::" + name, this);
  this->cameras.push_back(camera);

  return camera;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of cameras in this scene
unsigned int Scene::GetCameraCount() const
{
  return this->cameras.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a specific camera by index
Camera *Scene::GetCamera(unsigned int index) const
{
  if (index < this->cameras.size())
    return this->cameras[index];

  return NULL;
}


////////////////////////////////////////////////////////////////////////////////
// Create a user camera
UserCamera *Scene::CreateUserCamera(const std::string &name)
{
  UserCamera *camera = new UserCamera(this->name + "::" + name, this);
  camera->Load(NULL);
  camera->Init();
  this->userCameras.push_back(camera);

  return camera;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of user cameras in this scene
unsigned int Scene::GetUserCameraCount() const
{
  return this->userCameras.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a specific user camera by index
UserCamera *Scene::GetUserCamera(unsigned int index) const
{
  if (index < this->userCameras.size())
    return this->userCameras[index];

  return NULL;
}


////////////////////////////////////////////////////////////////////////////////
/// Get an entity at a pixel location using a camera. Used for mouse picking. 
Common *Scene::GetEntityAt(Camera *camera, Vector2<int> mousePos, std::string &mod) 
{
  /* NATY: put back in
  Common *entity = NULL;
  Ogre::Camera *ogreCam = camera->GetCamera();
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
      this->GetMeshInformation( pentity->getMesh(), vertex_count, 
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

    Visual* const* vis = Ogre::any_cast<Visual*>(&closestEntity->getUserAny());

    if (vis && (*vis)->GetOwner())
    {
      entity = (*vis)->GetOwner();
      return entity;
    }
  }

*/
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the world pos of a the first contact at a pixel location
Vector3 Scene::GetFirstContact(Camera *camera, Vector2<int> mousePos)
{
  Ogre::Camera *ogreCam = camera->GetCamera();
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

// NATY
/*
////////////////////////////////////////////////////////////////////////////////
// Register a camera
void Scene::RegisterCamera( Camera *cam )
{
  this->cameras.push_back(cam);
}

////////////////////////////////////////////////////////////////////////////////
/// Unregister a user camera
void Scene::UnregisterCamera( Camera *cam )
{
  std::vector<Camera*>::iterator iter;
  for(iter=this->cameras.begin();iter != this->cameras.end();iter++)
  {
    if ((*iter) == cam)
    {
      this->cameras.erase(iter);
      break;
    }
  }
}
*/

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
  Ogre::SceneNode *snode = dynamic_cast<Ogre::SceneNode*>(node);

  std::string nodeName = node->getName();
  int numAttachedObjs = 0;
  bool isInSceneGraph = false;
  if (snode)
  {
    numAttachedObjs = snode->numAttachedObjects();
    isInSceneGraph = snode->isInSceneGraph();
  }
  int numChildren = node->numChildren();
  Ogre::Vector3 pos = node->getPosition();
  Ogre::Vector3 scale = node->getScale();

  std::cout << prefix << nodeName << "\n";
  std::cout << prefix << "  Num Objs[" << numAttachedObjs << "]\n";
  std::cout << prefix << "  Num Children[" << numChildren << "]\n";
  std::cout << prefix << "  IsInGraph[" << isInSceneGraph << "]\n";
  std::cout << prefix << "  Pos[" << pos.x << " " << pos.y << " " << pos.z << "]\n";
  std::cout << prefix << "  Scale[" << scale.x << " " << scale.y << " " << scale.z << "]\n";
  
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
void Scene::InitShadows()
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

////////////////////////////////////////////////////////////////////////////////
// Get the scene ID
unsigned int Scene::GetId() const
{
  return this->id;
}

////////////////////////////////////////////////////////////////////////////////
// Get the scene Id as a string
std::string Scene::GetIdString() const
{
  return this->idString;
}

////////////////////////////////////////////////////////////////////////////////
/// Receive a message
void Scene::ReceiveMessage( const google::protobuf::Message &message)
{
  /*boost::mutex::scoped_lock lock( this->mutex );
  bool find = false;

  for (unsigned int i=0; i < this->messages[msg.type].size(); i++)
  {
    if (this->messages[msg.type][i]->id == msg.id)
    {
      delete this->messages[msg.type][i];
      this->messages[msg.type][i] = msg.Clone();
      find = true;
      break;
    }
  }
     
  if (!find) 
    this->messages[msg.type].push_back( msg.Clone() );
    */
}

////////////////////////////////////////////////////////////////////////////////
// Process all messages
void Scene::ProcessMessages()
{
  /*boost::mutex::scoped_lock lock( this->mutex );
  std::vector<Message*>::iterator iter;

  for (iter =  this->messages[VISUAL_MSG].begin(); 
       iter != this->messages[VISUAL_MSG].end(); iter++)
  {
    this->HandleVisualMsg((VisualMsg*)*iter);
  }

  for (iter =  this->messages[LIGHT_MSG].begin(); 
       iter != this->messages[LIGHT_MSG].end(); iter++)
  {
    this->HandleLightMsg((LightMsg*)*iter);
  }

  for (iter =  this->messages[SELECTION_MSG].begin(); 
       iter != this->messages[SELECTION_MSG].end(); iter++)
  {
    this->HandleSelectionMsg((SelectionMsg*)*iter);
  }

  for (iter =  this->messages[POSE_MSG].begin(); 
       iter != this->messages[POSE_MSG].end(); iter++)
  {
    PoseMsg *pmsg = (PoseMsg*)*iter;
    VisualMap::iterator iter2;
    iter2 = this->visuals.find(pmsg->id);

    if (iter2 != this->visuals.end())
    {
      iter2->second->SetPose( pmsg->pose );
    }
  }

  this->messages.clear();
  */
}

////////////////////////////////////////////////////////////////////////////////
// Get the mesh information for the given mesh.
// Code found in Wiki: www.ogre3d.org/wiki/index.php/RetrieveVertexData
void Scene::GetMeshInformation(const Ogre::MeshPtr mesh,
                               size_t &vertex_count,
                               Ogre::Vector3* &vertices,
                               size_t &index_count,
                               unsigned long* &indices,
                               const Ogre::Vector3 &position,
                               const Ogre::Quaternion &orient,
                               const Ogre::Vector3 &scale)
{
  bool added_shared = false;
  size_t current_offset = 0;
  size_t shared_offset = 0;
  size_t next_offset = 0;
  size_t index_offset = 0;

  vertex_count = index_count = 0;

  // Calculate how many vertices and indices we're going to need
  for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
  {
    Ogre::SubMesh* submesh = mesh->getSubMesh( i );

    // We only need to add the shared vertices once
    if(submesh->useSharedVertices)
    {
      if( !added_shared )
      {
        vertex_count += mesh->sharedVertexData->vertexCount;
        added_shared = true;
      }
    }
    else
    {
      vertex_count += submesh->vertexData->vertexCount;
    }

    // Add the indices
    index_count += submesh->indexData->indexCount;
  }


  // Allocate space for the vertices and indices
  vertices = new Ogre::Vector3[vertex_count];
  indices = new unsigned long[index_count];

  added_shared = false;

  // Run through the submeshes again, adding the data into the arrays
  for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
  {
    Ogre::SubMesh* submesh = mesh->getSubMesh(i);

    Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

    if((!submesh->useSharedVertices)||(submesh->useSharedVertices && !added_shared))
    {
      if(submesh->useSharedVertices)
      {
        added_shared = true;
        shared_offset = current_offset;
      }

      const Ogre::VertexElement* posElem =
        vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

      Ogre::HardwareVertexBufferSharedPtr vbuf =
        vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

      unsigned char* vertex =
        static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

      // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
      //  as second argument. So make it float, to avoid trouble when Ogre::Real will
      //  be comiled/typedefed as double:
      //      Ogre::Real* pReal;
      float* pReal;

      for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
      {
        posElem->baseVertexPointerToElement(vertex, &pReal);

        Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);

        vertices[current_offset + j] = (orient * (pt * scale)) + position;
      }

      vbuf->unlock();
      next_offset += vertex_data->vertexCount;
    }


    Ogre::IndexData* index_data = submesh->indexData;
    size_t numTris = index_data->indexCount / 3;
    Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

    bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

    unsigned long*  pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
    unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);


    size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;

    // Ogre 1.6 patch (commenting the static_cast...) - index offsets start from 0 for each submesh
    if ( use32bitindexes )
    {
      for ( size_t k = 0; k < numTris*3; ++k)
      {
        indices[index_offset++] = pLong[k] /*+ static_cast<unsigned long>(offset)*/;
      }
    }
    else
    {
      for ( size_t k = 0; k < numTris*3; ++k)
      {
        indices[index_offset++] = static_cast<unsigned long>(pShort[k]) /*+
                                                                          static_cast<unsigned long>(offset)*/;
      }
    }

    ibuf->unlock();
    current_offset = next_offset;
  }
}

void Scene::HandleVisualMsg(const msgs::Visual &msg)
{
  /*
  VisualMap::iterator iter;
  iter = this->visuals.find(msg->id);

  if (msg->action == VisualMsg::DELETE)
  {
    if (iter != this->visuals.end() )
    {
      delete iter->second;
      iter->second = NULL;
      this->visuals.erase(iter);
    }
  }
  else if (iter != this->visuals.end())
  {
    //if (msg->id == "line1")
      //printf("Updating a line1msg\n");
    iter->second->UpdateFromMsg(msg);
    //if (msg->id == "line1")
     // printf("Done Updating a line1msg\n");
  }
  else
  {
    if (msg->id == "line1")
      printf("Creating a line1msg\n");

    Visual *visual = NULL;
    VisualMap::iterator iter;
    iter = this->visuals.find(msg->parentId);

    if (iter != this->visuals.end())
      visual = new Visual(msg->id, iter->second);
    else 
      visual = new Visual(msg->id, this);
    visual->LoadFromMsg(msg);
    std::cout << "Creating visual[" << msg->id << "]\n";
    this->visuals[msg->id] = visual;

    if (msg->id == "line1")
      printf("Done Creating\n");
  }
  */
}

void Scene::HandleLightMsg(const msgs::Light &msg)
{
  /*
  LightMap::iterator iter;
  iter = this->lights.find(msg->id);

  if (msg->action == LightMsg::DELETE)
  {
    printf("DELETE  A LIGHT!!\n");
  }
  else if (iter != this->lights.end())
  {
    printf(" UPDATE A LIGHT\n");
  }
  else
  {
    Light *light = new Light(this);
    light->LoadFromMsg(msg);
    this->lights[msg->id] = light;
  }
  */
}

void Scene::HandleSelectionMsg(const msgs::Selection &msg)
{
  /*VisualMap::iterator viter;
  viter = this->visuals.find(msg->id);
  if (viter != this->visuals.end())
    viter->second->ShowSelectionBox(msg->selected);
    */
}
