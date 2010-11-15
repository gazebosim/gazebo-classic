#include <iostream>

#include "Camera.hh"
#include "MouseEvent.hh"
#include "Simulator.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "World.hh"
#include "PointLightMaker.hh"

using namespace gazebo;

PointLightMaker::PointLightMaker()
  : EntityMaker()
{
  this->state = 0;
  this->lightName = "";
  this->index = 0;
}

PointLightMaker::~PointLightMaker()
{
}

void PointLightMaker::Start()
{
  std::ostringstream stream;
  std::string name = "user_light";

  do
  {
    stream.str("");
    stream << name << index;
    this->index++;
  } while (Common::GetByName(stream.str()));

  this->lightName = stream.str();
  this->state = 1;
}

void PointLightMaker::Stop()
{
  this->state = 0;
}

bool PointLightMaker::IsActive() const
{
  return this->state > 0;
}

void PointLightMaker::MousePushCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  Vector3 norm;
  norm.Set(0,0,1);

  this->createPos = event.camera->GetWorldPointOnPlane(event.pressPos.x, event.pressPos.y, norm, 0);
}

void PointLightMaker::MouseReleaseCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->state++;

  this->CreateTheEntity();
  this->Start();
}

void PointLightMaker::MouseDragCB(const MouseEvent & /*event*/)
{
}

void PointLightMaker::CreateTheEntity()
{
  std::ostringstream newModelStr;

  newModelStr << "<?xml version='1.0'?> <gazebo:world xmlns:xi='http://www.w3.org/2001/XInclude' xmlns:gazebo='http://playerstage.sourceforge.net/gazebo/xmlschema/#gz' xmlns:model='http://playerstage.sourceforge.net/gazebo/xmlschema/#model' xmlns:sensor='http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor' xmlns:body='http://playerstage.sourceforge.net/gazebo/xmlschema/#body' xmlns:geom='http://playerstage.sourceforge.net/gazebo/xmlschema/#geom' xmlns:joint='http://playerstage.sourceforge.net/gazebo/xmlschema/#joint' xmlns:interface='http://playerstage.sourceforge.net/gazebo/xmlschema/#interface' xmlns:rendering='http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering' xmlns:renderable='http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable' xmlns:controller='http://playerstage.sourceforge.net/gazebo/xmlschema/#controller' xmlns:physics='http://playerstage.sourceforge.net/gazebo/xmlschema/#physics' >";

  newModelStr << "<model:renderable name=\"" << this->lightName << "\">\
    <xyz>" << this->createPos.x << " " << this->createPos.y << " " << 0.2 << "</xyz>\
    <static>true</static>\
    <light>\
      <type>point</type>\
      <specularColor>0.1 0.1 0.1</specularColor>\
      <diffuseColor>0.5 0.5 0.5</diffuseColor>\
      <attenuation>0.5 0.01 0.001</attenuation>\
      <range>20</range>\
    </light>\
    </model:renderable>";

  newModelStr <<  "</gazebo:world>";

  Simulator::Instance()->GetActiveWorld()->InsertEntity(newModelStr.str());
}
