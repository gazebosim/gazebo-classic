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
/* Desc: Some functions that creates Ogre objects together
 * Author: Jordi Polo
 * Date: 27 Dec 2007
 */

#include <Ogre.h>

#include <math.h>
#include <iostream>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <FL/Fl.H>
#include <FL/x.H>

#include "config.h"

#include "RTShaderSystem.hh"
#include "Light.hh"
#include "Material.hh"
#include "Simulator.hh"
#include "Global.hh"
#include "Entity.hh"
#include "XMLConfig.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "OgreMovableText.hh"
#include "OgreAdaptor.hh"
#include "OgreVisual.hh"
#include "OgreDynamicLines.hh"
#include "OgreCreator.hh"

using namespace gazebo;

unsigned int OgreCreator::windowCounter = 0;

////////////////////////////////////////////////////////////////////////////////
// Constructor
OgreCreator::OgreCreator()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
OgreCreator::~OgreCreator()
{
}

////////////////////////////////////////////////////////////////////////////////
// Create a plane
std::string OgreCreator::CreatePlane(const Vector3 &normal, 
    const Vector2<double> &size, const Vector2<double> &segments, 
    const Vector2<double> &uvTile, const std::string &material, 
    bool castShadows, OgreVisual *parent, const std::string &name)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return std::string();

  Vector3 n = normal;
  std::string resultName;

  n.Normalize();
  Vector3 perp = n.GetPerpendicular();

  Ogre::Plane plane(Ogre::Vector3(n.x, n.y, n.z), 0);

//FIXME: only one plane per parent
//TODO:names and parents

  if (name.empty())
    resultName = parent->GetName() + "_PLANE";
  else
    resultName = name;

  while (!Ogre::MeshManager::getSingleton().getByName(resultName).isNull())
    resultName += "A";

  try
  {
    Ogre::MeshManager::getSingleton().createPlane(resultName,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
        size.x, size.y,
        (int)segments.x, (int)segments.y,
        true,1,
        uvTile.x, uvTile.y,
        Ogre::Vector3(perp.x, perp.y, perp.z));

    parent->AttachMesh(resultName);
    parent->SetMaterial(material);

    parent->SetCastShadows(castShadows);
  }
  catch (Ogre::ItemIdentityException e)
  {
    std::cerr << "Error creating plane\n";
  }

  return resultName;
}


////////////////////////////////////////////////////////////////////////////////
/// Create a light source and attach it to the visual node
Light *OgreCreator::CreateLight(Entity *parent)
{
  Light *newLight = new Light(parent);
  this->lights.push_back(newLight);
  return newLight;
}

////////////////////////////////////////////////////////////////////////////////
// Remove a light
void OgreCreator::DeleteLight(Light *light)
{
  if (!light)
    return;

  std::list<Light*>::iterator iter;
  for (iter = this->lights.begin(); iter != this->lights.end(); iter++)
  {
    if (*iter == light)
    {
      delete *iter;
      this->lights.erase(iter);
      break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Helper function to create a camera
Ogre::Camera *OgreCreator::CreateCamera(const std::string &name, 
    double nearClip, double farClip, double hfov, 
    Ogre::RenderTarget *renderTarget)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return NULL;

  Ogre::Camera *camera;
  Ogre::Viewport *cviewport;

  camera = OgreAdaptor::Instance()->sceneMgr->createCamera(name);

  // Use X/Y as horizon, Z up
  camera->pitch(Ogre::Degree(90));

  // Don't yaw along variable axis, causes leaning
  camera->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);

  camera->setDirection(1,0,0);

  camera->setNearClipDistance(nearClip);
  camera->setFarClipDistance(farClip);

  if (renderTarget)
  {
    // Setup the viewport to use the texture
    cviewport = renderTarget->addViewport(camera);
    cviewport->setClearEveryFrame(true);
    cviewport->setBackgroundColour( *OgreAdaptor::Instance()->backgroundColor );

    double ratio = (double)cviewport->getActualWidth() / (double)cviewport->getActualHeight();
    double vfov = 2.0 * atan(tan(hfov / 2.0) / ratio);
    camera->setAspectRatio(ratio);
    camera->setFOVy(Ogre::Radian(vfov));

    RTShaderSystem::Instance()->AttachViewport(cviewport);
  }

  return camera;
}

////////////////////////////////////////////////////////////////////////////////
// Helper function to delete a camera
void OgreCreator::DeleteCamera(Ogre::Camera* camera)
{
  OgreAdaptor::Instance()->sceneMgr->destroyCamera(camera);
}

////////////////////////////////////////////////////////////////////////////////
void OgreCreator::CreateFog(XMLConfigNode *cnode)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  if (cnode)
  {
    Ogre::ColourValue backgroundColor;
    Ogre::FogMode fogType = Ogre::FOG_NONE;
    std::string type;
    double density;
    double linearStart, linearEnd;

    backgroundColor.r = cnode->GetTupleDouble("color",0,0);
    backgroundColor.g = cnode->GetTupleDouble("color",1,0);
    backgroundColor.b = cnode->GetTupleDouble("color",2,0);
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

    if (type != "none")
    {
      OgreAdaptor::Instance()->sceneMgr->setFog(fogType, backgroundColor, density, linearStart, linearEnd);
      //OgreAdaptor::Instance()->sceneMgr->setFog(Ogre::FOG_LINEAR, backgroundColor, 0, linearStart, linearEnd);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void OgreCreator::SaveFog(std::string &prefix, std::ostream &stream)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  Ogre::ColourValue color=OgreAdaptor::Instance()->sceneMgr->getFogColour();
  Ogre::Real start = OgreAdaptor::Instance()->sceneMgr->getFogStart();
  Ogre::Real end = OgreAdaptor::Instance()->sceneMgr->getFogEnd();
  Ogre::Real density = OgreAdaptor::Instance()->sceneMgr->getFogDensity();
  std::string fogMode="";

  switch (OgreAdaptor::Instance()->sceneMgr->getFogMode())
  {
    case Ogre::FOG_EXP:
      fogMode="exp";
      break;
    case Ogre::FOG_EXP2:
      fogMode="exp2";
      break;
    case Ogre::FOG_LINEAR:
      //case default:
      fogMode="linear";
      break;
    case Ogre::FOG_NONE:
      fogMode="none";
      break;
  }

  stream << prefix << "  <fog>\n";
  stream << prefix << "    <type>" << fogMode << "</type>\n";
  stream << prefix << "    <color>" << color.r << " " << color.g << " " << color.b << " " << color.a << "</color>\n";
  stream << prefix << "    <linearStart>" << start << "</linearStart>\n";
  stream << prefix << "    <linearEnd>" << end << "</linearEnd>\n";
  stream << prefix << "    <density>" << density << "</density>\n";
  stream << prefix << "  </fog>\n";
}

////////////////////////////////////////////////////////////////////////////////
// Create a sky
void OgreCreator::CreateSky(std::string material)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  if (!material.empty())
  {
    try
    {
      /*if (node->GetChild("fog"))
        {
        Ogre::Plane plane;
        plane.d = 49;
        plane.normal = Ogre::Vector3::NEGATIVE_UNIT_Z;
        OgreAdaptor::Instance()->sceneMgr->setSkyPlane(true, plane, material, 500, 100, true, 0.5, 150, 150);
        }
        else
        {*/
      Ogre::Quaternion orientation;
      orientation.FromAngleAxis( Ogre::Degree(90), Ogre::Vector3(1,0,0));
      //OgreAdaptor::Instance()->sceneMgr->setSkyDome(true,material,5,8, 4000, true, orientation);
      OgreAdaptor::Instance()->sceneMgr->setSkyDome(true,material,10,8, 4, true, orientation);
      //}

    }
    catch (int)
    {
      gzmsg(0) << "Unable to set sky dome to material[" << material << "]\n";
    }

  }
}


////////////////////////////////////////////////////////////////////////////////
/// Draw a named line
void OgreCreator::DrawLine(const Vector3 &start, const Vector3 &end, 
                           const std::string &name)
{
  Ogre::SceneNode *node = NULL;
  Ogre::ManualObject *obj = NULL;
  bool attached = false;

  if ( OgreAdaptor::Instance()->sceneMgr->hasManualObject(name))
  {
    node = OgreAdaptor::Instance()->sceneMgr->getSceneNode(name);
    obj = OgreAdaptor::Instance()->sceneMgr->getManualObject(name);
    attached = true;
  }
  else
  {
    node = OgreAdaptor::Instance()->sceneMgr->getRootSceneNode()->createChildSceneNode(name);
    obj = OgreAdaptor::Instance()->sceneMgr->createManualObject(name); 
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
/// Hide a visual
void OgreCreator::SetVisible(const std::string &name, bool visible)
{
  if (OgreAdaptor::Instance()->sceneMgr->hasSceneNode(name))
    OgreAdaptor::Instance()->sceneMgr->getSceneNode(name)->setVisible(visible);

  if ( OgreAdaptor::Instance()->sceneMgr->hasManualObject(name))
    OgreAdaptor::Instance()->sceneMgr->getManualObject(name)->setVisible(visible);
}

////////////////////////////////////////////////////////////////////////////////
/// Remove a mesh by name
void OgreCreator::RemoveMesh(const std::string &name)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  if (!name.empty() && Ogre::MeshManager::getSingleton().resourceExists(name))
    Ogre::MeshManager::getSingleton().remove(name);
}

////////////////////////////////////////////////////////////////////////////////
// Create a window for Ogre
Ogre::RenderWindow *OgreCreator::CreateWindow(Fl_Window *flWindow, unsigned int width, unsigned int height)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return NULL;

  Ogre::RenderWindow *win = NULL;

  if (flWindow)
  {
    XSync(fl_display, false);

    win = OgreCreator::CreateWindow( fl_display, fl_visual->screen, 
        (int32_t)(Fl_X::i(flWindow)->xid), width, height);
    if (win)
      this->windows.push_back(win);
  }

  return win;
}

////////////////////////////////////////////////////////////////////////////////
// Create a window for Ogre
Ogre::RenderWindow *OgreCreator::CreateWindow(Display *display, int screen, 
                                              int32_t winId, unsigned int width,                                              unsigned int height)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return NULL;

  std::stringstream ogreHandle;

  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;
  Ogre::RenderWindow *window = NULL;

  /*std::string screenStr = DisplayString((long)display);
  std::string::size_type dotPos = screenStr.find(".");
  screenStr = screenStr.substr(dotPos+1, screenStr.size());

  int attrList[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, 16, 
                    GLX_STENCIL_SIZE, 16, None };
  XVisualInfo *vi = glXChooseVisual(display, DefaultScreen((long)display), 
                                    attrList);

  ogreHandle << (unsigned long)display 
             << ":" << screenStr 
             << ":" << (unsigned long)winId 
             << ":" << (unsigned long)vi;
  */

  ogreHandle << winId;

  params["parentWindowHandle"] = ogreHandle.str();

  std::ostringstream stream;
  stream << "OgreWindow(" << windowCounter++ << ")";

  int attempts = 0;
  while (window == NULL && (attempts++) < 10)
  {
    try
    {
      window = OgreAdaptor::Instance()->root->createRenderWindow( stream.str(), 
                     width, height, false, &params);
    }
    catch (...)
    {
      //gzerr(0) << " Unable to create the rendering window\n";
      window = NULL;
    }
  }

  if (attempts >= 10)
  {
    gzthrow("Unable to create the rendering window\n");
  }

  window->setActive(true);
  window->setVisible(true);
  window->setAutoUpdated(true);

  this->windows.push_back(window);

  return window;
}

////////////////////////////////////////////////////////////////////////////////
// Create a material from a color definition
std::string OgreCreator::CreateMaterial(float r, float g, float b, float a)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return std::string();


  std::ostringstream matNameStream;

  matNameStream << "Color[" << r << "," << g << "," << b << "," << a << "]";

  if (!Ogre::MaterialManager::getSingleton().resourceExists(matNameStream.str()))
  {
    Ogre::MaterialPtr matPtr = Ogre::MaterialManager::getSingleton().create(
        matNameStream.str(),"General");

    matPtr->getTechnique(0)->setLightingEnabled(true);
    matPtr->getTechnique(0)->getPass(0)->setDiffuse(r,g,b,a);
    matPtr->getTechnique(0)->getPass(0)->setAmbient(r,g,b);
//    matPtr->getTechnique(0)->getPass(0)->setSelfIllumination(r,g,b);
  //  matPtr->setReceiveShadows(false);
  }

  return matNameStream.str();
}

////////////////////////////////////////////////////////////////////////////////
/// Create a material from a gazebo material
std::string OgreCreator::CreateMaterial(const Material *mat)
{
  if (Ogre::MaterialManager::getSingleton().resourceExists(mat->GetName()))
    return mat->GetName();

  Ogre::MaterialPtr matPtr = Ogre::MaterialManager::getSingleton().create(
      mat->GetName(),"General");

  Ogre::Pass *pass = matPtr->getTechnique(0)->getPass(0);

  Color ambient = mat->GetAmbient();
  Color diffuse = mat->GetDiffuse();
  Color specular = mat->GetSpecular();
  Color emissive = mat->GetEmissive();

  matPtr->getTechnique(0)->setLightingEnabled(true);
  pass->setDiffuse(diffuse.R(), diffuse.G(), diffuse.B(), diffuse.A());
  pass->setAmbient(ambient.R(), ambient.G(), ambient.B());
  pass->setPointSize(mat->GetPointSize());

  pass->setSpecular(specular.R(), specular.G(), specular.B(), specular.A());
  pass->setSelfIllumination(emissive.R(), emissive.G(), emissive.B());
  pass->setShininess(mat->GetShininess());

  if (!mat->GetTextureImage().empty())
  {
    Ogre::TextureUnitState *texState = pass->createTextureUnitState();
    texState->setTextureName( mat->GetTextureImage() );
  }

  return mat->GetName();
}

////////////////////////////////////////////////////////////////////////////////
/// Create a material from a texture file
std::string OgreCreator::CreateMaterialFromTexFile(const std::string &filename)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return std::string();

  if (!Ogre::MaterialManager::getSingleton().resourceExists(filename))
  {
    Ogre::MaterialPtr matPtr = Ogre::MaterialManager::getSingleton().create(
        filename, "General");
    Ogre::Pass *pass = matPtr->getTechnique(0)->createPass();

    //matPtr->setLightingEnabled(false);

    //pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    //pass->setDepthWriteEnabled(false);
    //pass->setDepthCheckEnabled(false);
    pass->createTextureUnitState(filename);
  }

  return filename;
}

////////////////////////////////////////////////////////////////////////////////
// Create a dynamic line
OgreDynamicLines *OgreCreator::CreateDynamicLine(OgreDynamicRenderable::OperationType opType)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return NULL;

  OgreDynamicLines *line = new OgreDynamicLines(opType);

  this->lines.push_back( line );

  return line;
}

////////////////////////////////////////////////////////////////////////////////
// Update all the entities
void OgreCreator::Update()
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  std::list<OgreDynamicLines*>::iterator iter;
  std::list<OgreMovableText*>::iterator titer;
  std::list<Ogre::RenderWindow*>::iterator witer;
  std::map<std::string, OgreVisual*>::iterator viter;

  // Update the text
  for (titer = this->text.begin(); titer != this->text.end(); titer++)
    (*titer)->Update();

  // Update the lines
  //for (iter = this->lines.begin(); iter != this->lines.end(); iter++)
    //(*iter)->Update();

  // We only need this loop because we are using threads. The physics engine
  // can't reliably set the pose of the visuals when it's running in a 
  // separate thread.
  if (!this->visuals.empty())
  {
    boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
    // Update the visuals
    for (viter = this->visuals.begin(); viter != this->visuals.end(); viter++)
    {
      if (viter->second)
      {
        // lock in case the visual is being dynamically destroyed
        boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMDMutex());
        if (!viter->second->IsDirty())
          continue;
        viter->second->SetToDirtyPose();
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Create a new ogre visual 
OgreVisual *OgreCreator::CreateVisual( const std::string &name,
    OgreVisual *parent, Entity *owner)
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return NULL;

  OgreVisual *newVis = NULL;
  std::map<std::string, OgreVisual*>::iterator iter;

  iter = this->visuals.find(name);

  if (iter == this->visuals.end())
  {
    newVis = new OgreVisual(parent, owner);
    newVis->SetName(name);

    this->visuals[name] = newVis;
  }
  else
    gzthrow(std::string("Name of ogre visual already exists: ") + name);

  return newVis;
}

////////////////////////////////////////////////////////////////////////////////
// Get a visual
OgreVisual *OgreCreator::GetVisual( const std::string &name )
{
  std::map<std::string, OgreVisual*>::iterator iter;
  iter = this->visuals.find(name);

  if (iter != this->visuals.end())
    return iter->second;

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Delete a visual
void OgreCreator::DeleteVisual( OgreVisual *visual )
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return;

  std::map<std::string, OgreVisual*>::iterator iter;

  iter = this->visuals.find(visual->GetName());

  if (iter != this->visuals.end() && iter->second != NULL)
  {
    delete iter->second;
    iter->second = NULL;
    this->visuals.erase(iter);
  }
  else
  {
    gzerr(0) << "Unknown visual[" << visual->GetName() << "]\n";
  }

}

////////////////////////////////////////////////////////////////////////////////
/// Delete a visual
void OgreCreator::DeleteVisual( const std::string &visname )
{
  this->DeleteVisual( this->GetVisual(visname) );
}

////////////////////////////////////////////////////////////////////////////////
/// Create a movable text object
OgreMovableText *OgreCreator::CreateMovableText()
{
  if (!Simulator::Instance()->GetRenderEngineEnabled())
    return NULL;

  OgreMovableText *newText = new OgreMovableText();
  this->text.push_back(newText);
  return newText;
}

////////////////////////////////////////////////////////////////////////////////
/// Insert a mesh into Ogre 
void OgreCreator::InsertMesh( const Mesh *mesh)
{
  Ogre::MeshPtr ogreMesh;

  if (mesh->GetSubMeshCount() == 0)
    return;

  try
  {
    // Create a new mesh specifically for manual definition.
    ogreMesh = Ogre::MeshManager::getSingleton().createManual(mesh->GetName(),
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    for (unsigned int i=0; i < mesh->GetSubMeshCount(); i++)
    {
      Ogre::SubMesh *ogreSubMesh;
      Ogre::VertexData *vertexData;
      Ogre::VertexDeclaration* vertexDecl;
      Ogre::HardwareVertexBufferSharedPtr vBuf;
      Ogre::HardwareIndexBufferSharedPtr iBuf;
      float *vertices;
      unsigned short *indices;


      size_t currOffset = 0;

      const SubMesh *subMesh = mesh->GetSubMesh(i);

      ogreSubMesh = ogreMesh->createSubMesh();
      ogreSubMesh->useSharedVertices = false;
      ogreSubMesh->vertexData = new Ogre::VertexData();
      vertexData = ogreSubMesh->vertexData;
      vertexDecl = vertexData->vertexDeclaration;

      // The vertexDecl should contain positions, blending weights, normals,
      // diffiuse colors, specular colors, tex coords. In that order.
      vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, 
                             Ogre::VES_POSITION);
      currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

      // TODO: blending weights
      
      // normals
      if (subMesh->GetNormalCount() > 0 )
      {
        vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, 
                               Ogre::VES_NORMAL);
        currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
      }

      // TODO: diffuse colors

      // TODO: specular colors

      // two dimensional texture coordinates
      if (subMesh->GetTexCoordCount() > 0)
      {
        vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2,
            Ogre::VES_TEXTURE_COORDINATES, 0);
        currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
      }

      // allocate the vertex buffer
      vertexData->vertexCount = subMesh->GetVertexCount();

      vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
                 vertexDecl->getVertexSize(0),
                 vertexData->vertexCount,
                 Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
                 false);

      vertexData->vertexBufferBinding->setBinding(0, vBuf);
      vertices = static_cast<float*>(vBuf->lock(
                      Ogre::HardwareBuffer::HBL_DISCARD));

      // allocate index buffer
      ogreSubMesh->indexData->indexCount = subMesh->GetIndexCount();

      ogreSubMesh->indexData->indexBuffer =
        Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
            Ogre::HardwareIndexBuffer::IT_16BIT,
            ogreSubMesh->indexData->indexCount,
            Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
            false);

      iBuf = ogreSubMesh->indexData->indexBuffer;
      indices = static_cast<unsigned short*>(
          iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

      unsigned int j;

      // Add all the vertices
      for (j =0; j < subMesh->GetVertexCount(); j++)
      {
        *vertices++ = subMesh->GetVertex(j).x;
        *vertices++ = subMesh->GetVertex(j).y;
        *vertices++ = subMesh->GetVertex(j).z;

        if (subMesh->GetNormalCount() > 0)
        {
          *vertices++ = subMesh->GetNormal(j).x;
          *vertices++ = subMesh->GetNormal(j).y;
          *vertices++ = subMesh->GetNormal(j).z;
        }

        if (subMesh->GetTexCoordCount() > 0)
        {
          *vertices++ = subMesh->GetTexCoord(j).x;
          *vertices++ = subMesh->GetTexCoord(j).y;
        }
      }

      // Add all the indices
      for (j =0; j < subMesh->GetIndexCount(); j++)
        *indices++ = subMesh->GetIndex(j);

      const Material *material;
      material = mesh->GetMaterial( subMesh->GetMaterialIndex() );
      if (material)
        ogreSubMesh->setMaterialName( OgreCreator::CreateMaterial( material ) );

      // Unlock
      vBuf->unlock();
      iBuf->unlock();
    }

    Vector3 max = mesh->GetMax();
    Vector3 min = mesh->GetMin();

    if (!max.IsFinite())
      gzthrow("Max bounding box is not finite[" << max << "]\n");

    if (!min.IsFinite())
      gzthrow("Min bounding box is not finite[" << min << "]\n");


    ogreMesh->_setBounds( Ogre::AxisAlignedBox(
          Ogre::Vector3(min.x, min.y, min.z),
          Ogre::Vector3(max.x, max.y, max.z)), 
          false );

    // this line makes clear the mesh is loaded (avoids memory leaks)
    ogreMesh->load();
  }
  catch (Ogre::Exception e)
  {
    gzerr(0) << "Unable to create a basic Unit cylinder object" << std::endl;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get the mesh information for the given mesh.
// Code found in Wiki: www.ogre3d.org/wiki/index.php/RetrieveVertexData
void OgreCreator::GetMeshInformation(const Ogre::MeshPtr mesh,
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

////////////////////////////////////////////////////////////////////////////////
/// Get the world bounding box for a visual
void OgreCreator::GetVisualBounds(OgreVisual *vis, Vector3 &min, Vector3 &max)
{
  Ogre::AxisAlignedBox box;

  Ogre::SceneNode *node = vis->GetSceneNode();
  GetSceneNodeBounds(node,box);

  min.x = box.getMinimum().x;
  min.y = box.getMinimum().y;
  min.z = box.getMinimum().z;

  max.x = box.getMaximum().x;
  max.y = box.getMaximum().y;
  max.z = box.getMaximum().z;
}

////////////////////////////////////////////////////////////////////////////////
// Get the bounding box for a scene node
void OgreCreator::GetSceneNodeBounds(Ogre::SceneNode *node, Ogre::AxisAlignedBox &box)
{
  node->_updateBounds();
  //box.merge(node->_getWorldAABB());
  Ogre::SceneNode::ChildNodeIterator it = node->getChildIterator();

  for (int i=0; i < node->numAttachedObjects(); i++)
  {
    Ogre::MovableObject *obj = node->getAttachedObject(i);
    if (obj->isVisible() && obj->getMovableType() != "gazebo::ogredynamiclines")
    {
      Ogre::Any any = obj->getUserAny();
      if (any.getType() == typeid(std::string))
      {
        std::string str = Ogre::any_cast<std::string>(any);
        if (str.substr(0,3) == "rot" || str.substr(0,5) == "trans")
          continue;
      }
      box.merge(obj->getWorldBoundingBox());
    }
  }

  while(it.hasMoreElements())
  {
    Ogre::SceneNode *next = dynamic_cast<Ogre::SceneNode*>(it.getNext());
    GetSceneNodeBounds( next, box);
  }
}
