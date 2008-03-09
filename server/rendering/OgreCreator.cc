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

#include <math.h>
#include <Ogre.h>
#include <iostream>

#include "Entity.hh"
#include "XMLConfig.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "MovableText.hh"
#include "OgreAdaptor.hh"
#include "OgreVisual.hh"
#include "OgreSimpleShape.hh"
#include "OgreCreator.hh"


using namespace gazebo;

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
void OgreCreator::CreateBasicShapes()
{
  // Create some basic shapes
  OgreSimpleShape::CreateSphere("unit_sphere",1.0, 32, 32);
  OgreSimpleShape::CreateSphere("joint_anchor",0.01, 32, 32);
  OgreSimpleShape::CreateBox("unit_box", Vector3(1,1,1));
  OgreSimpleShape::CreateCylinder("unit_cylinder", 0.5, 1.0, 1, 32);
}

////////////////////////////////////////////////////////////////////////////////
// Create a plane
OgreVisual *OgreCreator::CreatePlane(XMLConfigNode *node, Entity *parent)
{
  Vector3 normal = node->GetVector3("normal",Vector3(0,1,0));
  Vector2<double> size = node->GetVector2d("size",Vector2<double>(1000, 1000));
  Vector2<double> segments = node->GetVector2d("segments",Vector2<double>(10, 10));
  Vector2<double> uvTile = node->GetVector2d("uvTile",Vector2<double>(1, 1));
  std::string material=node->GetString("material","",1);


  normal.Normalize();
  Vector3 perp = normal.GetPerpendicular();

  Ogre::Plane plane(Ogre::Vector3(normal.x, normal.y, normal.z), 0);

  Ogre::MeshManager::getSingleton().createPlane(parent->GetUniqueName(),
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
      size.x, size.y, 
      (int)segments.x, (int)segments.y,
      true,1,
      uvTile.x, uvTile.y, 
      Ogre::Vector3(perp.x, perp.y, perp.z));

  OgreVisual *visual = new OgreVisual(parent->GetVisualNode());
  visual->AttachMesh(parent->GetUniqueName());
  visual->SetMaterial(material);
  visual->SetCastShadows(node->GetBool("castShadows",true,0));

  return visual;
}



////////////////////////////////////////////////////////////////////////////////
/// Create a light source and attach it to the entity
void OgreCreator::CreateLight(XMLConfigNode *node, Entity *entity)
{
  Vector3 vec;
  double range,constant,linear,quad;

  // Create the light
  Ogre::Light *light(OgreAdaptor::Instance()->sceneMgr->createLight(entity->GetUniqueName()));

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
  //  options for spotlights
  if (lightType == "spot")
  {
    vec = node->GetVector3("range", Vector3(5.0, 10.0, 1.0));
    light->setSpotlightRange(Ogre::Radian(Ogre::Degree(vec.x)), Ogre::Radian(Ogre::Degree(vec.y)), vec.z);
  }

  entity->GetVisualNode()->AttachObject(light);
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
  camera->setFarClipDistance(farClip+1000);

  // Setup the viewport to use the texture
  cviewport = renderTarget->addViewport(camera);
  cviewport->setClearEveryFrame(true);
  cviewport->setBackgroundColour( *OgreAdaptor::Instance()->backgroundColor );
  cviewport->setOverlaysEnabled(false);

  double ratio = (double)cviewport->getActualWidth() / (double)cviewport->getActualHeight();
  double vfov = 2.0 * atan(tan(hfov / 2.0) / ratio);
  camera->setAspectRatio(ratio);
  camera->setFOVy(Ogre::Radian(vfov));

  return camera;
}


////////////////////////////////////////////////////////////////////////////////
void OgreCreator::CreateFog(XMLConfigNode *node)
{
  Ogre::ColourValue backgroundColor;
  XMLConfigNode *cnode;

  if ((cnode = node->GetChild("fog")))
  {
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
void OgreCreator::SaveFog(XMLConfigNode *node)
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
  }
  node->SetValue("type", fogMode);
  node->SetValue("color", &color);
  node->SetValue("linearStart", start);
  node->SetValue("linearEnd", end);
  //node->SetValue("density", density);

}

////////////////////////////////////////////////////////////////////////////////
// Create a sky
void OgreCreator::CreateSky(XMLConfigNode *node)
{
  XMLConfigNode *cnode;
  if ((cnode = node->GetChild("sky")))
  {
    std::string material = cnode->GetString("material","",1);
    if (!material.empty())
    {
      try
      {
        if (node->GetChild("fog"))
        {
          Ogre::Plane plane;
          plane.d = 49;
          plane.normal = Ogre::Vector3::NEGATIVE_UNIT_Z;
          OgreAdaptor::Instance()->sceneMgr->setSkyPlane(true, plane, material, 50, 8, true, 0.5, 150, 150);
        }
        else
        {
          Ogre::Quaternion orientation;
          orientation.FromAngleAxis( Ogre::Degree(90), Ogre::Vector3(1,0,0));
          OgreAdaptor::Instance()->sceneMgr->setSkyDome(true,material,5,8, 4000, true, orientation);
        }

      }
      catch (int)
      {
        gzmsg(0) << "Unable to set sky dome to material[" << material << "]\n";
      }

    }
  }
}


////////////////////////////////////////////////////////////////////////////////
// Draw a grid on the ground
void OgreCreator::DrawGrid()
{
  Ogre::ManualObject* gridObject =  OgreAdaptor::Instance()->sceneMgr->createManualObject("__OGRE_GRID__"); 

  Ogre::SceneNode* gridObjectNode = OgreAdaptor::Instance()->sceneMgr->getRootSceneNode()->createChildSceneNode("__OGRE_GRID_NODE__"); 

  Ogre::MaterialPtr gridObjectMaterialX = Ogre::MaterialManager::getSingleton().create("__OGRE_GRID_MATERIAL_X__","debugger1"); 
  gridObjectMaterialX->setReceiveShadows(true); 
  gridObjectMaterialX->getTechnique(0)->setLightingEnabled(true); 
  gridObjectMaterialX->getTechnique(0)->getPass(0)->setDiffuse(0.4,0.0,0.0,0); 
  gridObjectMaterialX->getTechnique(0)->getPass(0)->setAmbient(0.4,0.0,0.0); 
  gridObjectMaterialX->getTechnique(0)->getPass(0)->setSelfIllumination(0.1,0.0,0.0); 

  Ogre::MaterialPtr gridObjectMaterialY = Ogre::MaterialManager::getSingleton().create("__OGRE_GRID_MATERIAL_Y__","debugger2"); 
  gridObjectMaterialY->setReceiveShadows(true); 
  gridObjectMaterialY->getTechnique(0)->setLightingEnabled(true); 
  gridObjectMaterialY->getTechnique(0)->getPass(0)->setDiffuse(0.0,0.0,0.4,0); 
  gridObjectMaterialY->getTechnique(0)->getPass(0)->setAmbient(0.0,0.0,0.4); 
  gridObjectMaterialY->getTechnique(0)->getPass(0)->setSelfIllumination(0.0,0.0,0.1); 


  float d = 0.01;
  float z = 0.01;
  
  gridObject->begin("__OGRE_GRID_MATERIAL_Y__", Ogre::RenderOperation::OT_TRIANGLE_LIST); 

  for (int y=-100; y<100; y++)
  {
    if (y%10 == 0)
      d = 0.04;
    else
      d = 0.01;

    gridObject->position(-100, y-d, z); 
    gridObject->position(100, y-d, z); 
    gridObject->position(100, y+d, z); 

    gridObject->position(-100, y-d, z); 
    gridObject->position(100, y+d, z); 
    gridObject->position(-100, y+d, z); 

    char *name=new char[20];
    char *text=new char[10];

    sprintf(name,"(%d %d)_yaxis",0,y);
    sprintf(text,"%d",y);

    MovableText* msg = new MovableText();
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
    msg->SetTextAlignment(MovableText::H_CENTER, MovableText::V_ABOVE);

    Ogre::SceneNode *textNode = OgreAdaptor::Instance()->sceneMgr->getRootSceneNode()->createChildSceneNode(std::string(name)+"_node");
 
    textNode->attachObject(msg); 
    textNode->translate(0, y, 0.02);

    delete name;
    delete text;

  }

  gridObject->end(); 
  gridObject->begin("__OGRE_GRID_MATERIAL_X__", Ogre::RenderOperation::OT_TRIANGLE_LIST); 

  z -= 0.001;

  for (int x=-100; x<100; x++)
  {
    if (x%10 == 0)
      d = 0.04;
    else
      d = 0.01;

    gridObject->position(x+d, 100, z); 
    gridObject->position(x-d, 100, z); 
    gridObject->position(x-d, -100, z); 

    gridObject->position(x+d, -100, z); 
    gridObject->position(x+d, 100, z); 
    gridObject->position(x-d, -100, z); 

    char *name=new char[20];
    char *text=new char[10];

    sprintf(name,"(%d %d)_xaxis",x,0);
    sprintf(text,"%d",x);

    MovableText* msg = new MovableText();
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
    msg->SetTextAlignment(MovableText::H_CENTER, MovableText::V_ABOVE);

    Ogre::SceneNode *textNode = OgreAdaptor::Instance()->sceneMgr->getRootSceneNode()->createChildSceneNode(std::string(name)+"_node");
 
    textNode->attachObject(msg); 
    textNode->translate(x, 0, 0.02);

    delete name;
    delete text;
  }
  
  // etc 
  gridObject->end(); 
  gridObjectNode->attachObject(gridObject);
}
