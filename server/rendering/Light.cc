#include <Ogre.h>

#include "OgreDynamicLines.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "OgreAdaptor.hh"
#include "XMLConfig.hh"
#include "GazeboError.hh"
#include "Global.hh"
#include "GazeboMessage.hh"
#include "Light.hh"

using namespace gazebo;

unsigned int Light::lightCounter = 0;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Light::Light(Entity *parent)
  : Entity(parent)
{
  std::ostringstream stream;

  stream << this->parent->GetName() << "_LIGHT" << this->lightCounter;
  this->SetName(stream.str());

  this->lightCounter++;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Light::~Light()
{
  delete this->line;
  delete this->visual;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the light
void Light::Load(XMLConfigNode *node)
{
  Vector3 vec;
  double range,constant,linear,quad;

  try
  {
    this->light = OgreAdaptor::Instance()->sceneMgr->createLight(
        this->GetName());
  }
  catch (Ogre::Exception e)
  {
    gzthrow("Ogre Error:" << e.getFullDescription() << "\n" << \
        "Unable to create a light on " + this->parent->GetName());
  }

  // Set the light type
  std::string lightType = node->GetString("type","point",0);
  if (lightType == "point")
  {
    this->light->setType(Ogre::Light::LT_POINT);
  }
  else if (lightType == "directional")
  {
    this->light->setType(Ogre::Light::LT_DIRECTIONAL);
  }
  else if (lightType == "spot")
  {
    this->light->setType(Ogre::Light::LT_SPOTLIGHT);
  }

  // Set the diffuse color
  vec = node->GetVector3("diffuseColor",Vector3(1.0, 1.0, 1.0));
  this->light->setDiffuseColour(vec.x, vec.y, vec.z);

  // Sets the specular color
  vec = node->GetVector3("specularColor",Vector3(1.0, 1.0, 1.0));
  this->light->setSpecularColour(vec.x, vec.y, vec.z);

  // Set the direction which the light points
  vec = node->GetVector3("direction", Vector3(0.0, 0.0, -1.0));
  vec.Normalize();
  this->light->setDirection(vec.x, vec.y, vec.z);

  // Absolute range of light in world coordinates
  range = node->GetDouble("range",0,100);

  // Constant factor. 1.0 means never attenuate, 0.0 is complete attenuation
  constant = node->GetTupleDouble("attenuation",0,1.0);
  if (constant < 0)
    constant = 0;
  else if (constant > 1.0)
    constant = 1.0;

  // Linear factor. 1 means attenuate evenly over the distance
  linear = node->GetTupleDouble("attenuation",1,0);
  if (linear < 0)
    linear = 0;
  else if (linear > 1.0)
    linear = 1.0;

  // Quadartic factor.adds a curvature to the attenuation formula
  quad = node->GetTupleDouble("attenuation",2,0);

  // Set attenuation
  this->light->setAttenuation(range, constant, linear, quad);

  // TODO: More options for Spot lights, etc.
  //  options for spotlights
  if (lightType == "spot")
  {
    vec = node->GetVector3("spotCone", Vector3(5.0, 10.0, 1.0));
    this->light->setSpotlightRange(Ogre::Radian(Ogre::Degree(vec.x)), 
        Ogre::Radian(Ogre::Degree(vec.y)), vec.z);
  }

  this->light->setCastShadows(node->GetBool("castShadows",true,0));

  this->parent->GetVisualNode()->AttachObject(light);

  this->CreateVisual();
}

////////////////////////////////////////////////////////////////////////////////
// Save a light
void Light::Save(const std::string &prefix, std::ostream &stream)
{
  std::string type;

  if (this->light->getType() == Ogre::Light::LT_POINT)
    type = "point";
  else if (this->light->getType() == Ogre::Light::LT_DIRECTIONAL)
    type = "directional";
  else 
    type = "spot";

  Ogre::ColourValue diffuseColor = this->light->getDiffuseColour();
  Ogre::ColourValue specularColor = this->light->getDiffuseColour();
  Ogre::Vector3 dir = this->light->getDirection();
  Ogre::Real attRange = this->light->getAttenuationRange();
  Ogre::Real attConst = this->light->getAttenuationConstant();
  Ogre::Real attLinear = this->light->getAttenuationLinear();
  Ogre::Real attQuadric = this->light->getAttenuationQuadric();

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

  stream << prefix << "  <range>"<< attRange << "</range>\n";

  stream << prefix << "  <attenuation>" << " " << attConst 
         << " " << attLinear << " " << attQuadric << "</attenuation>\n";
  stream << prefix << "</light>\n";
}


////////////////////////////////////////////////////////////////////////////////
// Helper node to create a visual representation of the light
void Light::CreateVisual()
{
  if (this->light->getType() == Ogre::Light::LT_DIRECTIONAL)
    return;

  this->visual = new OgreVisual(this->parent->GetVisualNode());

  // The lines draw a visualization of the camera
  this->line = OgreCreator::Instance()->CreateDynamicLine(
      OgreDynamicRenderable::OT_LINE_LIST);

  float s=0.1;
  this->line->AddPoint(Vector3(-s,-s,0));
  this->line->AddPoint(Vector3(-s,s,0));

  this->line->AddPoint(Vector3(-s,s,0));
  this->line->AddPoint(Vector3(s,s,0));

  this->line->AddPoint(Vector3(s,s,0));
  this->line->AddPoint(Vector3(s,-s,0));

  this->line->AddPoint(Vector3(s,-s,0));
  this->line->AddPoint(Vector3(-s,-s,0));



  this->line->AddPoint(Vector3(-s,-s,0));
  this->line->AddPoint(Vector3(0,0,s));

  this->line->AddPoint(Vector3(-s,s,0));
  this->line->AddPoint(Vector3(0,0,s));

  this->line->AddPoint(Vector3(s,s,0));
  this->line->AddPoint(Vector3(0,0,s));

  this->line->AddPoint(Vector3(s,-s,0));
  this->line->AddPoint(Vector3(0,0,s));



  this->line->AddPoint(Vector3(-s,-s,0));
  this->line->AddPoint(Vector3(0,0,-s));

  this->line->AddPoint(Vector3(-s,s,0));
  this->line->AddPoint(Vector3(0,0,-s));

  this->line->AddPoint(Vector3(s,s,0));
  this->line->AddPoint(Vector3(0,0,-s));

  this->line->AddPoint(Vector3(s,-s,0));
  this->line->AddPoint(Vector3(0,0,-s));

  this->line->setMaterial("Gazebo/WhiteEmissive");
  this->line->setVisibilityFlags(GZ_LASER_CAMERA);

  this->visual->AttachObject(line);
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether this entity has been selected by the user through the gui
bool Light::SetSelected( bool s )
{
  Entity::SetSelected(s);

  if (s)
    this->line->setMaterial("Gazebo/PurpleEmissive");
  else
    this->line->setMaterial("Gazebo/WhiteEmissive");
}


