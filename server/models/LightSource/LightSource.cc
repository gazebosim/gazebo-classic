#include <Ogre.h>
#include "OgreAdaptor.hh"
#include "ModelFactory.hh"
#include "XMLConfig.hh"
#include "LightSource.hh"

using namespace gazebo;

GZ_REGISTER_STATIC("LightSource", LightSource);

LightSource::LightSource()
{
}

LightSource::~LightSource()
{
}

// Load the child model
int LightSource::LoadChild(XMLConfigNode *node)
{
  Vector3 vec;
  double range,constant,linear,quad;
  char *lightname = new char[50];
  sprintf(lightname,"light%d",this->GetId());

  // Create the light
  Ogre::Light *light = OgreAdaptor::Instance()->sceneMgr->createLight(lightname);
 // light->setType( Ogre::Light::LT_POINT);
  //light->setPosition(Ogre::Vector3(0,10.5,-10));
  light->setDiffuseColour(1.0, 1.0, 1.0);
  light->setSpecularColour(1.0, 1.0, 1.0);


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

  // Set the light position
  vec = node->GetVector3("xyz",Vector3(0, 0, 0));
  light->setPosition(Ogre::Vector3(vec.x,vec.y,vec.z));

  // Set the diffuse color
  vec = node->GetVector3("diffuseColor",Vector3(1.0, 1.0, 1.0)); 
  light->setDiffuseColour(vec.x,vec.y, vec.z);

  // Sets the specular color
  vec = node->GetVector3("specularColor",Vector3(1.0, 1.0, 1.0)); 
  light->setSpecularColour(vec.x, vec.y, vec.z);

  // Set the direction which the light points
  vec = node->GetVector3("direction", Vector3(0.0, 0.0, 0.0));
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
  
  delete [] lightname;
  return 0;
}

// Initialize the child model
int LightSource::InitChild()
{
  return 0;
}

// Update the child model
int LightSource::UpdateChild()
{
  return 0;
}

// Finilaize thie child model
int LightSource::FiniChild()
{
  return 0;
}
