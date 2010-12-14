#include <iostream>

#include "Camera.hh"
#include "MouseEvent.hh"
#include "Simulator.hh"
#include "Visual.hh"
#include "World.hh"
#include "DirectionalLightMaker.hh"

using namespace gazebo;

unsigned int DirectionalLightMaker::counter = 0;

DirectionalLightMaker::DirectionalLightMaker()
  : EntityMaker()
{
  this->state = 0;
  this->lightName = "";
}

DirectionalLightMaker::~DirectionalLightMaker()
{
}

void DirectionalLightMaker::Start(Scene *scene)
{
  std::ostringstream stream;
  stream << "user_directional_light_" << counter++;
  this->lightName = stream.str();
  this->state = 1;
}

void DirectionalLightMaker::Stop()
{
  this->state = 0;
}

bool DirectionalLightMaker::IsActive() const
{
  return this->state > 0;
}

void DirectionalLightMaker::MousePushCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  Vector3 norm;
  norm.Set(0,0,1);

  this->createPos = event.camera->GetWorldPointOnPlane(event.pressPos.x, event.pressPos.y, norm, 0);
}

void DirectionalLightMaker::MouseReleaseCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->state++;

  this->CreateTheEntity();
  this->Stop();
}

void DirectionalLightMaker::MouseDragCB(const MouseEvent & /*event*/)
{
}

void DirectionalLightMaker::CreateTheEntity()
{
  std::ostringstream newModelStr;

  newModelStr << "<?xml version='1.0'?> <gazebo:world xmlns:xi='http://www.w3.org/2001/XInclude' xmlns:gazebo='http://playerstage.sourceforge.net/gazebo/xmlschema/#gz' xmlns:model='http://playerstage.sourceforge.net/gazebo/xmlschema/#model' xmlns:sensor='http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor' xmlns:body='http://playerstage.sourceforge.net/gazebo/xmlschema/#body' xmlns:geom='http://playerstage.sourceforge.net/gazebo/xmlschema/#geom' xmlns:joint='http://playerstage.sourceforge.net/gazebo/xmlschema/#joint' xmlns:interface='http://playerstage.sourceforge.net/gazebo/xmlschema/#interface' xmlns:rendering='http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering' xmlns:renderable='http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable' xmlns:controller='http://playerstage.sourceforge.net/gazebo/xmlschema/#controller' xmlns:physics='http://playerstage.sourceforge.net/gazebo/xmlschema/#physics' >";

  newModelStr << "<model:renderable name=\"" << this->lightName << "\">\
    <xyz>" << this->createPos.x << " " << this->createPos.y << " " << 0.1 << "</xyz>\
    <static>true</static>\
    <light>\
      <type>directional</type>\
      <direction>.1 .1 -.9</direction>\
      <specularColor>0.1 0.1 0.1</specularColor>\
      <diffuseColor>0.8 0.8 0.8</diffuseColor>\
      <attenuation>0.5 0.01 0</attenuation>\
      <range>20</range>\
    </light>\
    </model:renderable>";

  newModelStr <<  "</gazebo:world>";

  Simulator::Instance()->GetActiveWorld()->InsertEntity(newModelStr.str());
}
