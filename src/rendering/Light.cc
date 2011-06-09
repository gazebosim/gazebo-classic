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

/* Desc: A Light
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

#include <boost/bind.hpp>

#include "rendering/ogre.h"

#include "common/Messages.hh"
#include "common/Events.hh"
#include "common/XMLConfig.hh"
#include "common/Exception.hh"
#include "common/Global.hh"
#include "common/Console.hh"

#include "rendering/Scene.hh"
#include "rendering/DynamicLines.hh"
#include "rendering/Visual.hh"
#include "rendering/Light.hh"

using namespace gazebo;
using namespace rendering;

unsigned int Light::lightCounter = 0;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Light::Light(Scene *scene)
{
  this->scene = scene;

  std::ostringstream stream;
  stream << "LIGHT" << this->lightCounter << "_VISUAL";
  this->visual = new Visual(stream.str(), this->scene);

  this->lightCounter++;

  common::Param::Begin(&this->parameters);
  this->nameP = new common::ParamT<std::string>("name","light",1);

  this->lightTypeP = new common::ParamT<std::string>("type", std::string("point"), 1);
  this->lightTypeP->Callback(&Light::SetLightType, this);

  this->diffuseP  = new common::ParamT<common::Color>("diffuse_color", common::Color(.5, .5, .5, 1), 0);
  this->diffuseP->Callback(&Light::SetDiffuseColor, this);

  this->specularP = new common::ParamT<common::Color>("specular_color", common::Color(.1, .1, .1), 0);
  this->specularP->Callback(&Light::SetSpecularColor, this);

  this->directionP  = new common::ParamT<common::Vector3>("direction", common::Vector3(0, 0, -1), 0);
  this->directionP->Callback(&Light::SetDirection, this);

  this->attenuationP  = new common::ParamT<common::Vector3>("attenuation", common::Vector3(.1, 0.01, .001), 1);
  this->attenuationP->Callback(&Light::SetAttenuation, this);

  this->spotInnerAngleP = new common::ParamT<double>("spot_inner_angle", 10, 0);
  this->spotInnerAngleP->Callback(&Light::SetSpotInnerAngle, this);

  this->spotOuterAngleP = new common::ParamT<double>("spot_outer_angle", 20, 0);
  this->spotOuterAngleP->Callback(&Light::SetSpotOuterAngle, this);

  this->spotFalloffP = new common::ParamT<double>("spot_falloff", 1, 0);
  this->spotFalloffP->Callback(&Light::SetSpotFalloff, this);

  this->rangeP  = new common::ParamT<double>("range",10,1);
  this->rangeP->Callback(&Light::SetRange, this);

  this->castShadowsP = new common::ParamT<bool>("cast_shadows",true,0);
  this->castShadowsP->Callback(&Light::SetCastShadows, this);
  common::Param::End();

  this->showLightsConnection = event::Events::ConnectShowLightsSignal(boost::bind(&Light::ToggleShowVisual, this));
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Light::~Light()
{
  if (this->light)
  {
    this->scene->GetManager()->destroyLight(this->GetName());
  }

  delete this->visual;
  delete this->lightTypeP;
  delete this->diffuseP;
  delete this->specularP;
  delete this->directionP;
  delete this->attenuationP;
  delete this->rangeP;
  delete this->castShadowsP;
  delete this->spotInnerAngleP;
  delete this->spotOuterAngleP;
  delete this->spotFalloffP;

}

////////////////////////////////////////////////////////////////////////////////
/// Load the light
void Light::Load(common::XMLConfigNode *node)
{
  common::Vector3 vec;

  if (node)
  {
    this->lightTypeP->Load(node);
    this->diffuseP->Load(node);
    this->specularP->Load(node);
    this->directionP->Load(node);
    this->attenuationP->Load(node);
    this->rangeP->Load(node);
    this->castShadowsP->Load(node);
    this->spotInnerAngleP->Load(node);
    this->spotOuterAngleP->Load(node);
    this->spotFalloffP->Load(node);
  }

  try
  {
    this->light = this->scene->GetManager()->createLight(this->GetName());
  }
  catch (Ogre::Exception e)
  {
    gzthrow("Ogre Error:" << e.getFullDescription() << "\n" << \
        "Unable to create a light");
  }

  this->SetLightType( **this->lightTypeP );
  this->SetDiffuseColor(**this->diffuseP);
  this->SetSpecularColor(**this->specularP);
  this->SetDirection(**this->directionP);
  this->SetAttenuation(**this->attenuationP);
  this->SetRange(**this->rangeP);
  this->SetCastShadows(**this->castShadowsP);
  this->SetSpotInnerAngle(**this->spotInnerAngleP);
  this->SetSpotOuterAngle(**this->spotOuterAngleP);
  this->SetSpotFalloff(**this->spotFalloffP);

  this->visual->AttachObject(this->light);

  this->CreateVisual();
  //this->SetupShadows();
}

////////////////////////////////////////////////////////////////////////////////
/// Load from a light message
void Light::LoadFromMsg(const boost::shared_ptr<msgs::Light const> &msg)
{
  this->nameP->SetValue( msg->header().str_id() );

  if (msg->has_type() && msg->type() == msgs::Light::POINT)
    this->lightTypeP->SetValue("point");
  else if (msg->has_type() && msg->type() == msgs::Light::SPOT)
    this->lightTypeP->SetValue("spot");
  else
    this->lightTypeP->SetValue("directional");

  if (msg->has_diffuse())
    this->diffuseP->SetValue(common::Message::Convert(msg->diffuse()));

  if (msg->has_specular())
    this->specularP->SetValue(common::Message::Convert(msg->specular()));

  if (msg->has_direction())
    this->directionP->SetValue(common::Message::Convert(msg->direction()));

  if (msg->has_attenuation())
    this->attenuationP->SetValue(common::Message::Convert(msg->attenuation()));

  if (msg->has_range())
    this->rangeP->SetValue(msg->range());

  if (msg->has_cast_shadows())
    this->castShadowsP->SetValue(msg->cast_shadows());

  if (msg->has_spot_inner_angle())
    this->spotInnerAngleP->SetValue(msg->spot_inner_angle());

  if (msg->has_spot_outer_angle())
    this->spotOuterAngleP->SetValue(msg->spot_outer_angle());

  if (msg->has_spot_falloff())
    this->spotFalloffP->SetValue(msg->spot_falloff());

  this->Load(NULL);

  if (msg->has_pose())
    this->SetPosition(common::Message::Convert(msg->pose().position()));
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
  stream << prefix << "  " << *(this->spotOuterAngleP) << "\n";
  stream << prefix << "  " << *(this->spotFalloffP) << "\n";
  stream << prefix << "  " << *(this->castShadowsP) << "\n";
  stream << prefix << "</light>\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Set the name of the visual
void Light::SetName( const std::string &name )
{
  this->nameP->SetValue(name);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of the visual
std::string Light::GetName() const
{
  return **this->nameP;
}

////////////////////////////////////////////////////////////////////////////////
// Helper node to create a visual representation of the light
void Light::CreateVisual()
{
  // The lines draw a visualization of the camera
  this->line = this->visual->AddDynamicLine( RENDERING_LINE_LIST );

  if ( **this->lightTypeP == "directional")
  {
    float s=.5;
    this->line->AddPoint( common::Vector3(-s, -s, 0) );
    this->line->AddPoint( common::Vector3(-s, s, 0) );

    this->line->AddPoint( common::Vector3(-s, s, 0) );
    this->line->AddPoint( common::Vector3(s, s, 0) );

    this->line->AddPoint( common::Vector3(s, s, 0) );
    this->line->AddPoint( common::Vector3(s, -s, 0) );

    this->line->AddPoint( common::Vector3(s, -s, 0) );
    this->line->AddPoint( common::Vector3(-s, -s, 0) );

    this->line->AddPoint( common::Vector3(0, 0, 0) );
    this->line->AddPoint( common::Vector3(0, 0, -s) );
  }
  if ( **this->lightTypeP == "point" )
  {
    float s=0.1;
    this->line->AddPoint(common::Vector3(-s,-s,0));
    this->line->AddPoint(common::Vector3(-s,s,0));

    this->line->AddPoint(common::Vector3(-s,s,0));
    this->line->AddPoint(common::Vector3(s,s,0));

    this->line->AddPoint(common::Vector3(s,s,0));
    this->line->AddPoint(common::Vector3(s,-s,0));

    this->line->AddPoint(common::Vector3(s,-s,0));
    this->line->AddPoint(common::Vector3(-s,-s,0));


    this->line->AddPoint(common::Vector3(-s,-s,0));
    this->line->AddPoint(common::Vector3(0,0,s));

    this->line->AddPoint(common::Vector3(-s,s,0));
    this->line->AddPoint(common::Vector3(0,0,s));

    this->line->AddPoint(common::Vector3(s,s,0));
    this->line->AddPoint(common::Vector3(0,0,s));

    this->line->AddPoint(common::Vector3(s,-s,0));
    this->line->AddPoint(common::Vector3(0,0,s));



    this->line->AddPoint(common::Vector3(-s,-s,0));
    this->line->AddPoint(common::Vector3(0,0,-s));

    this->line->AddPoint(common::Vector3(-s,s,0));
    this->line->AddPoint(common::Vector3(0,0,-s));

    this->line->AddPoint(common::Vector3(s,s,0));
    this->line->AddPoint(common::Vector3(0,0,-s));

    this->line->AddPoint(common::Vector3(s,-s,0));
    this->line->AddPoint(common::Vector3(0,0,-s));
  }
  else if ( this->light->getType() == Ogre::Light::LT_SPOTLIGHT )
  {
    double innerAngle = this->light->getSpotlightInnerAngle().valueRadians();
    double outerAngle = this->light->getSpotlightOuterAngle().valueRadians();

    double angles[2];
    double range = 0.2;
    angles[0] = range * tan(outerAngle);
    angles[1] = range * tan(innerAngle);

    unsigned int i = 0;
    this->line->AddPoint(common::Vector3(0,0,0));
    this->line->AddPoint(common::Vector3(angles[i],angles[i], -range));
 
    for (unsigned int i=0; i < 2; i++)
    {
      this->line->AddPoint(common::Vector3(0,0,0));
      this->line->AddPoint(common::Vector3(angles[i],angles[i], -range));

      this->line->AddPoint(common::Vector3(0,0,0));
      this->line->AddPoint(common::Vector3(-angles[i],-angles[i], -range));

      this->line->AddPoint(common::Vector3(0,0,0));
      this->line->AddPoint(common::Vector3(angles[i],-angles[i], -range));

      this->line->AddPoint(common::Vector3(0,0,0));
      this->line->AddPoint(common::Vector3(-angles[i],angles[i], -range));

      this->line->AddPoint(common::Vector3(angles[i],angles[i], -range));
      this->line->AddPoint(common::Vector3(-angles[i],angles[i], -range));

      this->line->AddPoint(common::Vector3(-angles[i],angles[i], -range));
      this->line->AddPoint(common::Vector3(-angles[i],-angles[i], -range));

      this->line->AddPoint(common::Vector3(-angles[i],-angles[i], -range));
      this->line->AddPoint(common::Vector3(angles[i],-angles[i], -range));

      this->line->AddPoint(common::Vector3(angles[i],-angles[i], -range));
      this->line->AddPoint(common::Vector3(angles[i],angles[i], -range));
    }
  }

  this->line->setMaterial("Gazebo/LightOn");
  this->line->setVisibilityFlags(GZ_LASER_CAMERA);

  // turn off light source box visuals by default
  this->visual->SetVisible(true);
}

////////////////////////////////////////////////////////////////////////////////
// Set the position of the light
void Light::SetPosition(const common::Vector3 &p)
{
  this->visual->SetPosition(p);
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether this entity has been selected by the user through the gui
bool Light::SetSelected( bool s )
{
  // NATY: FIX 
  // Entity::SetSelected(s);

  if (this->light->getType() != Ogre::Light::LT_DIRECTIONAL)
  {
    if (s)
      this->line->setMaterial("Gazebo/PurpleGlow");
    else
      this->line->setMaterial("Gazebo/LightOn");
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Toggle light visual visibility
void Light::ToggleShowVisual()
{
  this->visual->ToggleVisible();
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
  }
  else if (type == "spot")
    this->light->setType(Ogre::Light::LT_SPOTLIGHT);

  if (**this->lightTypeP != type)
    this->lightTypeP->SetValue( type );
}

////////////////////////////////////////////////////////////////////////////////
/// Set the diffuse
void Light::SetDiffuseColor(const common::Color &color)
{
  if (**this->diffuseP != color)
    this->diffuseP->SetValue( color );

  this->light->setDiffuseColour(color.R(), color.G(), color.B());
}

////////////////////////////////////////////////////////////////////////////////
/// Set the specular color
void Light::SetSpecularColor(const common::Color &color)
{
  if (**this->specularP != color)
    this->specularP->SetValue( color );

  this->light->setSpecularColour(color.R(), color.G(), color.B());
}

////////////////////////////////////////////////////////////////////////////////
/// Set the direction
void Light::SetDirection(const common::Vector3 &dir)
{
  // Set the direction which the light points
  common::Vector3 vec = dir;
  vec.Normalize();

  if (**this->directionP != vec)
    this->directionP->SetValue( vec );

  this->light->setDirection(vec.x, vec.y, vec.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the attenuation
void Light::SetAttenuation(const common::Vector3 &att)
{
  common::Vector3 vec = att;

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
/// Set cast shadows
void Light::SetCastShadows(const bool &cast)
{
  this->light->setCastShadows(true);
  /*if (this->light->getType() == Ogre::Light::LT_POINT)
    this->light->setCastShadows(false);
  else
  {
    if (**this->castShadowsP != cast)
      this->castShadowsP->SetValue( cast );

    this->light->setCastShadows(cast);
  }
  */
}

////////////////////////////////////////////////////////////////////////////////
/// Set the spot light inner angle
void Light::SetSpotInnerAngle(const double &angle)
{
  if (**this->spotInnerAngleP != angle)
    this->spotInnerAngleP->SetValue( angle );

  if (this->light->getType() == Ogre::Light::LT_SPOTLIGHT)
    this->light->setSpotlightRange(
        Ogre::Degree(**this->spotInnerAngleP), 
        Ogre::Degree(**this->spotOuterAngleP), 
        **this->spotFalloffP);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the spot light outter angle
void Light::SetSpotOuterAngle(const double &angle)
{
  if (**this->spotOuterAngleP != angle)
    this->spotOuterAngleP->SetValue( angle );

  if (this->light->getType() == Ogre::Light::LT_SPOTLIGHT)
    this->light->setSpotlightRange(
        Ogre::Degree(**this->spotInnerAngleP), 
        Ogre::Degree(**this->spotOuterAngleP), 
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
        Ogre::Degree(**this->spotInnerAngleP), 
        Ogre::Degree(**this->spotOuterAngleP), 
        **this->spotFalloffP);

}

////////////////////////////////////////////////////////////////////////////////
// Setup the shadow camera for the light
void Light::SetupShadows()
{
  if (this->light->getType() == Ogre::Light::LT_DIRECTIONAL)
  {
    unsigned int numShadowTextures = 3;

    // shadow camera setup
    Ogre::PSSMShadowCameraSetup* pssmSetup = new Ogre::PSSMShadowCameraSetup();

    Ogre::PSSMShadowCameraSetup::SplitPointList splitPointList = pssmSetup->getSplitPoints();

    // These were hand tuned by me (Nate)...hopefully they work for all cases.
    splitPointList[0] = 0.5;
    splitPointList[1] = 5.5;
    splitPointList[2] = 20.0;

    pssmSetup->setSplitPoints(splitPointList);
    pssmSetup->setSplitPadding(5.0);
    pssmSetup->setUseSimpleOptimalAdjust(true);

    // set the LISPM adjustment factor (see API documentation for these)
    /*pssmSetup->setOptimalAdjustFactor(0, 5.0);
    pssmSetup->setOptimalAdjustFactor(1, 3.0);
    pssmSetup->setOptimalAdjustFactor(2, 1.0);
    */

    this->light->setCustomShadowCameraSetup(Ogre::ShadowCameraSetupPtr(pssmSetup));

    Ogre::Vector4 splitPoints;
    for (unsigned int i = 0; i < numShadowTextures; ++i)
      splitPoints[i] = splitPointList[i];

    Ogre::MaterialManager::ResourceMapIterator iter = Ogre::MaterialManager::getSingleton().getResourceIterator();

    // Iterate over all the materials, and set the pssm split points
    while(iter.hasMoreElements())
    {
      Ogre::MaterialPtr mat = iter.getNext();
      for(int i = 0; i < mat->getNumTechniques(); i++) 
      {
        Ogre::Technique *tech = mat->getTechnique(i);
        for(int j = 0; j < tech->getNumPasses(); j++) 
        {
          Ogre::Pass *pass = tech->getPass(j);
          if (pass->hasFragmentProgram())
          {
            Ogre::GpuProgramParametersSharedPtr params = pass->getFragmentProgramParameters();
            if (params->_findNamedConstantDefinition("pssm_split_points"))
              params->setNamedConstant("pssm_split_points", splitPoints);
          }
        }
      }
    }
  }
  else if (this->light->getType() == Ogre::Light::LT_SPOTLIGHT)
  {
    this->light->setCustomShadowCameraSetup(Ogre::ShadowCameraSetupPtr(new Ogre::DefaultShadowCameraSetup()));
  }
}
