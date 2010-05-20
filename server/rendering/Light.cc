#include <Ogre.h>
#include <boost/bind.hpp>

#include "World.hh"
#include "Model.hh"
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
  this->type = Entity::LIGHT;

  std::ostringstream stream;

  stream << this->parent->GetName() << "_LIGHT" << this->lightCounter;
  this->SetName(stream.str());

  this->lightCounter++;

  Param::Begin(&this->parameters);
  this->lightTypeP = new ParamT<std::string>("type", std::string("point"), 1);
  this->lightTypeP->Callback(&Light::SetLightType, this);

  this->diffuseP  = new ParamT<Vector3>("diffuseColor", Vector3(.5, .5, .5), 0);
  this->diffuseP->Callback(&Light::SetDiffuseColor, this);

  this->specularP = new ParamT<Vector3>("specularColor", Vector3(.1, .1, .1), 0);
  this->specularP->Callback(&Light::SetSpecularColor, this);

  this->directionP  = new ParamT<Vector3>("direction", Vector3(0, 0, -1), 0);
  this->directionP->Callback(&Light::SetDirection, this);

  this->attenuationP  = new ParamT<Vector3>("attenuation", Vector3(.1, 0.01, .001), 1);
  this->attenuationP->Callback(&Light::SetAttenuation, this);

  this->spotInnerAngleP = new ParamT<double>("innerAngle", 10, 0);
  this->spotInnerAngleP->Callback(&Light::SetSpotInnerAngle, this);

  this->spotOutterAngleP = new ParamT<double>("outterAngle", 5, 0);
  this->spotOutterAngleP->Callback(&Light::SetSpotOutterAngle, this);

  this->spotFalloffP = new ParamT<double>("falloff", 1, 0);
  this->spotFalloffP->Callback(&Light::SetSpotFalloff, this);

  this->rangeP  = new ParamT<double>("range",10,1);
  this->rangeP->Callback(&Light::SetRange, this);

  this->castShadowsP = new ParamT<bool>("castShadows",true,0);
  this->castShadowsP->Callback(&Light::SetCastShadows, this);
  Param::End();


  World::Instance()->ConnectShowLightsSignal( boost::bind(&Light::ShowVisual, this, _1) );
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Light::~Light()
{
  delete this->line;
  delete this->visual;

  delete this->lightTypeP;
  delete this->diffuseP;
  delete this->specularP;
  delete this->directionP;
  delete this->attenuationP;
  delete this->rangeP;
  delete this->castShadowsP;
  delete this->spotInnerAngleP;
  delete this->spotOutterAngleP;
  delete this->spotFalloffP;
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

  this->lightTypeP->Load(node);
  this->diffuseP->Load(node);
  this->specularP->Load(node);
  this->directionP->Load(node);
  this->attenuationP->Load(node);
  this->rangeP->Load(node);
  this->castShadowsP->Load(node);
  this->spotInnerAngleP->Load(node);
  this->spotOutterAngleP->Load(node);
  this->spotFalloffP->Load(node);

  this->SetLightType( **this->lightTypeP );
  this->SetDiffuseColor(**this->diffuseP);
  this->SetSpecularColor(**this->specularP);
  this->SetDirection(**this->directionP);
  this->SetAttenuation(**this->attenuationP);
  this->SetRange(**this->rangeP);
  this->SetCastShadows(**this->castShadowsP);
  this->SetSpotInnerAngle(**this->spotInnerAngleP);
  this->SetSpotOutterAngle(**this->spotOutterAngleP);
  this->SetSpotFalloff(**this->spotFalloffP);

  // TODO: More options for Spot lights, etc.
  //  options for spotlights
  /*if ((**this->lightTypeP) == "spot")
  {
    vec = node->GetVector3("spotCone", Vector3(5.0, 10.0, 1.0));
    this->light->setSpotlightRange(Ogre::Radian(Ogre::Degree(vec.x)), 
        Ogre::Radian(Ogre::Degree(vec.y)), vec.z);
  }*/

  this->parent->GetVisualNode()->AttachObject(light);

  this->CreateVisual();
}

////////////////////////////////////////////////////////////////////////////////
// Save a light
void Light::Save(const std::string &prefix, std::ostream &stream)
{
  stream << prefix << "<light>\n";
  stream << prefix << "  " << *(this->lightTypeP) << "\n";
  stream << prefix << "  " << *(this->directionP) << "\n";
  stream << prefix << "  " << *(this->diffuseP) << "\n";
  stream << prefix << "  " << *(this->specularP) << "\n";
  stream << prefix << "  " << *(this->rangeP) << "\n";
  stream << prefix << "  " << *(this->attenuationP) << "\n";
  stream << prefix << "  " << *(this->spotInnerAngleP) << "\n";
  stream << prefix << "  " << *(this->spotOutterAngleP) << "\n";
  stream << prefix << "  " << *(this->spotFalloffP) << "\n";
  stream << prefix << "  " << *(this->castShadowsP) << "\n";
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

  // turn off light source box visuals by default
  this->visual->SetVisible(true);
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether this entity has been selected by the user through the gui
bool Light::SetSelected( bool s )
{
  Entity::SetSelected(s);

  if (this->light->getType() != Ogre::Light::LT_DIRECTIONAL)
  {
    if (s)
      this->line->setMaterial("Gazebo/PurpleEmissive");
    else
      this->line->setMaterial("Gazebo/WhiteEmissive");
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Set whether to show the visual
void Light::ShowVisual(bool s)
{
  this->visual->SetVisible(s);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the light type
void Light::SetLightType(const std::string &type)
{
  // Set the light type
  if (type == "point")
    this->light->setType(Ogre::Light::LT_POINT);
  else if (type == "directional")
  {
    this->light->setType(Ogre::Light::LT_DIRECTIONAL);
    this->parent->GetParentModel()->SetStatic(true);
  }
  else if (type == "spot")
    this->light->setType(Ogre::Light::LT_SPOTLIGHT);

  if (**this->lightTypeP != type)
    this->lightTypeP->SetValue( type );
}

////////////////////////////////////////////////////////////////////////////////
/// Set the diffuse
void Light::SetDiffuseColor(const Vector3 &color)
{
  if (**this->diffuseP != color)
    this->diffuseP->SetValue( color );

  this->light->setDiffuseColour(color.x, color.y, color.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the specular color
void Light::SetSpecularColor(const Vector3 &color)
{
  if (**this->specularP != color)
    this->specularP->SetValue( color );

  this->light->setSpecularColour(color.x, color.y, color.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the direction
void Light::SetDirection(const Vector3 &dir)
{
  // Set the direction which the light points
  Vector3 vec = dir;
  vec.Normalize();

  if (**this->directionP != vec)
    this->directionP->SetValue( vec );

  this->light->setDirection(vec.x, vec.y, vec.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the attenuation
void Light::SetAttenuation(const Vector3 &att)
{
  Vector3 vec = att;

  // Constant factor. 1.0 means never attenuate, 0.0 is complete attenuation
  if (vec.x < 0)
    vec.x = 0;
  else if (vec.x > 1.0)
    vec.x = 1.0;

  // Linear factor. 1 means attenuate evenly over the distance
  if (vec.y < 0)
    vec.y = 0;
  else if (vec.y > 1.0)
    vec.y = 1.0;

  if (**this->attenuationP != vec)
    this->attenuationP->SetValue( vec );

  // Set attenuation
  this->light->setAttenuation((**this->rangeP), vec.x, vec.y, vec.z);
}


////////////////////////////////////////////////////////////////////////////////
/// Set the range
void Light::SetRange(const double &range)
{
  if (**this->rangeP != range)
    this->rangeP->SetValue( range );

  this->light->setAttenuation(range, (**this->attenuationP).x, 
      (**this->attenuationP).y, (**this->attenuationP).z);
}

////////////////////////////////////////////////////////////////////////////////
/// Set cast shadowsj
void Light::SetCastShadows(const bool &cast)
{
  if (**this->castShadowsP != cast)
    this->castShadowsP->SetValue( cast );

  this->light->setCastShadows(cast);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the spot light inner angle
void Light::SetSpotInnerAngle(const double &angle)
{
  if (**this->spotInnerAngleP != angle)
    this->spotInnerAngleP->SetValue( angle );

  if (this->light->getType() == Ogre::Light::LT_SPOTLIGHT)
    this->light->setSpotlightRange(
        Ogre::Radian(Ogre::Degree(**this->spotInnerAngleP)), 
        Ogre::Radian(Ogre::Degree(**this->spotOutterAngleP)), 
        **this->spotFalloffP);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the spot light outter angle
void Light::SetSpotOutterAngle(const double &angle)
{
  if (**this->spotOutterAngleP != angle)
    this->spotOutterAngleP->SetValue( angle );

  if (this->light->getType() == Ogre::Light::LT_SPOTLIGHT)
    this->light->setSpotlightRange(
        Ogre::Radian(Ogre::Degree(**this->spotInnerAngleP)), 
        Ogre::Radian(Ogre::Degree(**this->spotOutterAngleP)), 
        **this->spotFalloffP);

}

////////////////////////////////////////////////////////////////////////////////
/// Set the spot light falloff
void Light::SetSpotFalloff(const double &angle)
{
  if (**this->spotFalloffP != angle)
    this->spotFalloffP->SetValue( angle );

  if (this->light->getType() == Ogre::Light::LT_SPOTLIGHT)
    this->light->setSpotlightRange(
        Ogre::Radian(Ogre::Degree(**this->spotInnerAngleP)), 
        Ogre::Radian(Ogre::Degree(**this->spotOutterAngleP)), 
        **this->spotFalloffP);

}
