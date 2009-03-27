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

#include "Simulator.hh"
#include "Geom.hh"
#include "Global.hh"
#include "Entity.hh"
#include "XMLConfig.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "OgreMovableText.hh"
#include "OgreAdaptor.hh"
#include "OgreVisual.hh"
#include "OgreDynamicLines.hh"
#include "OgreSimpleShape.hh"
#include "OgreCreator.hh"

using namespace gazebo;

unsigned int OgreCreator::lightCounter = 0;
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
// Create the basic shapes
void OgreCreator::LoadBasicShapes()
{
  // Create some basic shapes
  OgreSimpleShape::CreateSphere("unit_sphere",1.0, 32, 32);
  OgreSimpleShape::CreateSphere("joint_anchor",0.01, 32, 32);
  OgreSimpleShape::CreateBox("unit_box_U1V1", Vector3(1,1,1), 
                             Vector2<double>(1,1));
  OgreSimpleShape::CreateCylinder("unit_cylinder", 0.5, 1.0, 1, 32);
}

////////////////////////////////////////////////////////////////////////////////
// Create a plane
std::string OgreCreator::CreatePlane(const Vector3 &normal, const Vector2<double> &size, const Vector2<double> &segments, const Vector2<double> &uvTile, const std::string &material, bool castShadows, OgreVisual *parent, const std::string &name)
{
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

  Ogre::MeshManager::getSingleton().createPlane(resultName,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
      size.x, size.y,
      (int)segments.x, (int)segments.y,
      true,1,
      uvTile.x, uvTile.y,
      Ogre::Vector3(perp.x, perp.y, perp.z));
  
  parent->AttachMesh(parent->GetName() + "_PLANE");
  parent->SetMaterial(material);

  parent->SetCastShadows(castShadows);

  return resultName;
}


////////////////////////////////////////////////////////////////////////////////
/// Create a light source and attach it to the visual node
/// Note that the properties here are not modified afterwards and thus, 
/// we don't need a Light class. 
std::string OgreCreator::CreateLight(XMLConfigNode *node, OgreVisual *parent)
{
  Vector3 vec;
  double range,constant,linear,quad;
  Ogre::Light *light;

  // Create the light
  std::ostringstream stream;
  stream << parent->GetName() << "_LIGHT" << lightCounter;
  lightCounter++;

  try
  {
    light = OgreAdaptor::Instance()->sceneMgr->createLight(stream.str());
  } 
  catch (Ogre::Exception e)
  {
    gzthrow("Ogre Error:" << e.getFullDescription() << "\n" << \
             "Unable to create a light on " + parent->GetName());
  }
  
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
  vec = node->GetVector3("direction", Vector3(0.0, 0.0, -1.0));
  //light->setDirection(vec.x, vec.y, vec.z);

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
  //  options for spotlights
  if (lightType == "spot")
  {
    vec = node->GetVector3("range", Vector3(5.0, 10.0, 1.0));
    light->setSpotlightRange(Ogre::Radian(Ogre::Degree(vec.x)), 
        Ogre::Radian(Ogre::Degree(vec.y)), vec.z);
  }

  parent->AttachObject(light);

  return stream.str();
}

////////////////////////////////////////////////////////////////////////////////
// Save a light's information in xml format
void OgreCreator::SaveLight(const std::string &prefix, 
                           const std::string lightName, std::ostream &stream)
{
  Ogre::Light *light = NULL;
  std::string type;

  if (!OgreAdaptor::Instance()->sceneMgr->hasLight(lightName))
  {
    gzerr(0) << "Unknown light[" << lightName << "]\n";
    return;
  }

  light = OgreAdaptor::Instance()->sceneMgr->getLight(lightName);

  if (light->getType() == Ogre::Light::LT_POINT)
    type = "point";
  else if (light->getType() == Ogre::Light::LT_DIRECTIONAL)
    type = "directional";
  else 
    type = "spot";

  Ogre::ColourValue diffuseColor = light->getDiffuseColour();
  Ogre::ColourValue specularColor = light->getDiffuseColour();
  Ogre::Vector3 dir = light->getDirection();
  Ogre::Real attRange = light->getAttenuationRange();
  Ogre::Real attConst = light->getAttenuationConstant();
  Ogre::Real attLinear = light->getAttenuationLinear();
  Ogre::Real attQuadric = light->getAttenuationQuadric();

  stream << prefix << "<light>\n";
  stream << prefix << "  <type>" << type << "</type>\n";

  stream << prefix << "  <direction>" << dir.x << " " << dir.y << " " 
         << dir.z << "</direction>\n";

  stream << prefix << "  <diffuseColor>" << diffuseColor.r << " " 
         << diffuseColor.g << " " << diffuseColor.b << " " << diffuseColor.a 
         << "</diffuseColor>\n";

  stream << prefix << "  <specularColor>" << specularColor.r << " " 
         << specularColor.g << " " << specularColor.b << " "
         << specularColor.a << "</specularColor>\n";

  stream << prefix << "  <attenuation>" << attRange << " " << attConst 
         << " " << attLinear << " " << attQuadric << "</attenuation>\n";
  stream << prefix << "</light>\n";
}


////////////////////////////////////////////////////////////////////////////////
// Helper function to create a camera
Ogre::Camera *OgreCreator::CreateCamera(const std::string &name, double nearClip, double farClip, double hfov, Ogre::RenderTarget *renderTarget)
{
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
    //cviewport->setBackgroundColour( *OgreAdaptor::Instance()->backgroundColor );
    cviewport->setBackgroundColour( Ogre::ColourValue::Blue );

    double ratio = (double)cviewport->getActualWidth() / (double)cviewport->getActualHeight();
    double vfov = 2.0 * atan(tan(hfov / 2.0) / ratio);
    camera->setAspectRatio(ratio);
    camera->setFOVy(Ogre::Radian(vfov));
  }

  return camera;
}

////////////////////////////////////////////////////////////////////////////////
void OgreCreator::CreateFog(XMLConfigNode *cnode)
{
  if (cnode)
  {
    Ogre::ColourValue backgroundColor;
    //Ogre::FogMode fogType = Ogre::FOG_NONE;
    //std::string type;
    //double density;
    double linearStart, linearEnd;

    backgroundColor.r = cnode->GetTupleDouble("color",0,0);
    backgroundColor.g = cnode->GetTupleDouble("color",1,0);
    backgroundColor.b = cnode->GetTupleDouble("color",2,0);
    //type = cnode->GetString("type","linear",0);
    //density = cnode->GetDouble("density",0,0);
    linearStart = cnode->GetDouble("linearStart",0,0);
    linearEnd = cnode->GetDouble("linearEnd",1.0,0);

    /*if (type == "linear")
      fogType = Ogre::FOG_LINEAR;
      else if (type == "exp")
      fogType = Ogre::FOG_EXP;
      else if (type == "exp2")
      fogType = Ogre::FOG_EXP2;
      */

    //OgreAdaptor::Instance()->sceneMgr->setFog(fogType, backgroundColor, density, linearStart, linearEnd);
    OgreAdaptor::Instance()->sceneMgr->setFog(Ogre::FOG_LINEAR, backgroundColor, 0, linearStart, linearEnd);
  }
}

////////////////////////////////////////////////////////////////////////////////
void OgreCreator::SaveFog(std::string &prefix, std::ostream &stream)
{
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
      OgreAdaptor::Instance()->sceneMgr->setSkyDome(true,material,5,8, 4000, true, orientation);
      //}

    }
    catch (int)
    {
      gzmsg(0) << "Unable to set sky dome to material[" << material << "]\n";
    }

  }
}


////////////////////////////////////////////////////////////////////////////////
// Draw a grid on the ground
void OgreCreator::DrawGrid()
{
  Ogre::ManualObject* gridObject =  OgreAdaptor::Instance()->sceneMgr->createManualObject("__OGRE_GRID__");

  gridObject->setCastShadows(false);

  Ogre::SceneNode* gridObjectNode = OgreAdaptor::Instance()->sceneMgr->getRootSceneNode()->createChildSceneNode("__OGRE_GRID_NODE__");

  Ogre::MaterialPtr gridObjectMaterialX = Ogre::MaterialManager::getSingleton().create("__OGRE_GRID_MATERIAL_X__","General");
  gridObjectMaterialX->getTechnique(0)->setLightingEnabled(true);
  gridObjectMaterialX->getTechnique(0)->getPass(0)->setDiffuse(0.2,0.2,0.2,1);
  gridObjectMaterialX->getTechnique(0)->getPass(0)->setAmbient(0.2,0.2,0.2);
  //gridObjectMaterialX->getTechnique(0)->getPass(0)->setSelfIllumination(0.0,0.0,0.0);
  gridObjectMaterialX->setReceiveShadows(false);

  Ogre::MaterialPtr gridObjectMaterialY = Ogre::MaterialManager::getSingleton().create("__OGRE_GRID_MATERIAL_Y__","General");
  gridObjectMaterialY->getTechnique(0)->setLightingEnabled(true);
  gridObjectMaterialY->getTechnique(0)->getPass(0)->setDiffuse(0.2,0.2,0.2,1);
  gridObjectMaterialY->getTechnique(0)->getPass(0)->setAmbient(0.2,0.2,0.2);
  //gridObjectMaterialY->getTechnique(0)->getPass(0)->setSelfIllumination(0.0,0.0,0.0);
  gridObjectMaterialY->setReceiveShadows(false);


  float d = 0.01;
  int dim = 50;

  // Vertex Values for a square box
  float v[8][3] =
  {
    {-1, -1, -1}, {+1, -1, -1}, {+1, +1, -1}, {-1, +1, -1},
    {-1, -1, +1}, {+1, -1, +1}, {+1, +1, +1}, {-1, +1, +1}
  };

  // Indices
  int ind[36] =
  {
    // Bottom Face
    0, 1, 2,
    2, 3, 0,

    // Top Face
    4, 5, 7,
    7, 5, 6,

    // Front Face
    0, 4, 7,
    7, 3, 0,

    // Back face
    5, 1, 6,
    6, 1, 2,

    // Left face
    0, 5, 4,
    0, 1, 5,

    // Right face
    3, 7, 6,
    6, 2, 3


  };

  gridObject->begin("__OGRE_GRID_MATERIAL_Y__", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  for (int y=-dim; y<dim; y++)
  {
    if (y%10 == 0)
      d = 0.04;
    else
      d = 0.01;

    // For each face
    for (int i = 0; i < 36; i++)
    {
      gridObject->position(  v[ind[i]][0] * dim, 
                           y+v[ind[i]][1] * 0.02, 
                             v[ind[i]][2] * 0.01 );
    }
    char *name=new char[20];
    char *text=new char[10];

    sprintf(name,"(%d %d)_yaxis",0,y);
    sprintf(text,"%d",y);

    OgreMovableText* msg = new OgreMovableText();
    try
    {
      msg->Load(name, text,"Arial",0.08);
    }
    catch (Ogre::Exception e)
    {
      std::ostringstream stream;
      stream <<  "Unable to create the text. " << e.getDescription() <<std::endl;
      gzthrow(stream.str() );
    }
    msg->SetTextAlignment(OgreMovableText::H_CENTER, OgreMovableText::V_ABOVE);

    Ogre::SceneNode *textNode = OgreAdaptor::Instance()->sceneMgr->getRootSceneNode()->createChildSceneNode(std::string(name)+"_node");

    textNode->attachObject(msg);
    textNode->translate(0, y, 0.02);

    delete [] name;
    delete [] text;

  }

  gridObject->end();

  gridObject->begin("__OGRE_GRID_MATERIAL_X__", Ogre::RenderOperation::OT_TRIANGLE_LIST);

  for (int x=-dim; x<dim; x++)
  {
    if (x%10 == 0)
      d = 0.04;
    else
      d = 0.01;

    // For each face
    for (int i = 0; i < 36; i++)
    {
      gridObject->position(x+v[ind[i]][0] * 0.02, 
                             v[ind[i]][1] * dim, 
                             v[ind[i]][2] * 0.01 );
    }

    char *name=new char[20];
    char *text=new char[10];

    sprintf(name,"(%d %d)_xaxis",x,0);
    sprintf(text,"%d",x);

    OgreMovableText* msg = new OgreMovableText();
    try
    {
      msg->Load(name, text,"Arial",0.08);
    }
    catch (Ogre::Exception e)
    {
      std::ostringstream stream;
      stream <<  "Unable to create the text. " << e.getDescription() <<std::endl;
      gzthrow(stream.str() );
    }
    msg->SetTextAlignment(OgreMovableText::H_CENTER, OgreMovableText::V_ABOVE);

    Ogre::SceneNode *textNode = OgreAdaptor::Instance()->sceneMgr->getRootSceneNode()->createChildSceneNode(std::string(name)+"_node");

    textNode->attachObject(msg);
    textNode->translate(x, 0, 0.02);

    delete [] name;
    delete [] text;
  }

  // etc
  gridObject->end();
  gridObjectNode->attachObject(gridObject);
  
}

////////////////////////////////////////////////////////////////////////////////
/// Remove a mesh by name
void OgreCreator::RemoveMesh(const std::string &name)
{
  if (!name.empty() && Ogre::MeshManager::getSingleton().resourceExists(name))
    Ogre::MeshManager::getSingleton().remove(name);
}

////////////////////////////////////////////////////////////////////////////////
// Create a window for Ogre
Ogre::RenderWindow *OgreCreator::CreateWindow(Fl_Window *flWindow, unsigned int width, unsigned int height)
{
  Ogre::RenderWindow *win = NULL;

  if (flWindow)
  {
    win = OgreCreator::CreateWindow( (long)fl_display, fl_visual->screen, 
        (long)(Fl_X::i(flWindow)->xid), width, height);
    if (win)
      this->windows.push_back(win);
  }

  return win;
}

////////////////////////////////////////////////////////////////////////////////
// Create a window for Ogre
Ogre::RenderWindow *OgreCreator::CreateWindow(long display, int screen, 
                                              long winId, unsigned int width, 
                                              unsigned int height)
{
  width = 800;
  height = 600;

  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;
  Ogre::RenderWindow *window = NULL;

  /// Ogre 1.4 required this method to create windows
  /*params["parentWindowHandle"] = Ogre::StringConverter::toString(display) + 
    ":" + Ogre::StringConverter::toString(screen) + 
    ":" + Ogre::StringConverter::toString(winId);
    */

  /// As of Ogre 1.6 this is the params method that makes a resizable window
  params["externalWindowHandle"] =  Ogre::StringConverter::toString(display) + 
    ":" + Ogre::StringConverter::toString(screen) + 
    ":" + Ogre::StringConverter::toString(winId) + 
    ":" + Ogre::StringConverter::toString(fl_visual);

  params["FSAA"] = "2";

  std::ostringstream stream;
  stream << "OgreWindow(" << windowCounter++ << ")";

  int attempts = 0;
  while (window == NULL && (attempts++) < 10)
  {
    try
    {
      window = OgreAdaptor::Instance()->root->createRenderWindow( stream.str(), width, height, false, &params);
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
  window->setAutoUpdated(true);

  this->windows.push_back(window);

  return window;
}

////////////////////////////////////////////////////////////////////////////////
// Create a material from a color definition
std::string OgreCreator::CreateMaterial(float r, float g, float b, float a)
{
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
/// Create a material from a texture file
std::string OgreCreator::CreateMaterialFromTexFile(const std::string &filename)
{
  if (!Ogre::MaterialManager::getSingleton().resourceExists(filename))
  {
    Ogre::MaterialPtr matPtr = Ogre::MaterialManager::getSingleton().create(
        filename, "General");
    Ogre::Pass *pass = matPtr->getTechnique(0)->createPass();

    matPtr->setLightingEnabled(false);

    //pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    //pass->setDepthWriteEnabled(false);
    pass->createTextureUnitState(filename);
  }

  return filename;
}

////////////////////////////////////////////////////////////////////////////////
// Create a dynamic line
OgreDynamicLines *OgreCreator::CreateDynamicLine(OgreDynamicRenderable::OperationType opType)
{
  OgreDynamicLines *line = new OgreDynamicLines(opType);

  this->lines.push_back( line );

  return line;
}

////////////////////////////////////////////////////////////////////////////////
// Update all the entities
void OgreCreator::Update()
{
  std::list<OgreDynamicLines*>::iterator iter;
  std::list<OgreMovableText*>::iterator titer;
  std::list<Ogre::RenderWindow*>::iterator witer;
  std::map<std::string, OgreVisual*>::iterator viter;

  OgreVisual *vis = NULL;
  Entity *owner = NULL;

  // Update the text
  for (titer = this->text.begin(); titer != this->text.end(); titer++)
  {
    (*titer)->Update();
  }

  // Update the lines
  for (iter = this->lines.begin(); iter != this->lines.end(); iter++)
  {
    (*iter)->Update();
  }

  {
    boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMutex());
    // Update the visuals
    for (viter = this->visuals.begin(); viter != this->visuals.end(); viter++)
    {
      vis = viter->second;
      owner = vis->GetOwner();
      if (!owner)
        continue;

      Geom *geom = dynamic_cast<Geom*>(owner);

      if (geom || !owner->GetParent())
        vis->SetPose(owner->GetPose());
      else
        vis->SetPose(owner->GetPose() - owner->GetParent()->GetPose());

    }
  }

  /*for (witer = this->windows.begin(); witer != this->windows.end(); witer++)
  {
    (*witer)->update();
  }
  */

}

////////////////////////////////////////////////////////////////////////////////
/// Create a new ogre visual 
OgreVisual *OgreCreator::CreateVisual( const std::string &name,
    OgreVisual *parent, Entity *owner)
{
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
/// Delete a visual
void OgreCreator::DeleteVisual( OgreVisual *visual )
{
  std::map<std::string, OgreVisual*>::iterator iter;

  iter = this->visuals.find(visual->GetName());

  if (iter != this->visuals.end())
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
/// Create a movable text object
OgreMovableText *OgreCreator::CreateMovableText()
{
  OgreMovableText *newText = new OgreMovableText();
  this->text.push_back(newText);
  return newText;
}

