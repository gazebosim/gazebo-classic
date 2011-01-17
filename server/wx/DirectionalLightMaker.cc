#include <iostream>

#include "Events.hh"
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

  this->msg.type = LightMsg::DIRECTIONAL;
  this->msg.diffuse.Set(0.2, 0.2, 0.2, 1);
  this->msg.specular.Set(0.01, 0.01, 0.01, 1);
  this->msg.attenuation.Set(0.5, 0.01, 0.001);
  this->msg.direction.Set( .1, .1, -0.9);
  this->msg.range = 20;
  this->msg.castShadows = true;
}

DirectionalLightMaker::~DirectionalLightMaker()
{
}

void DirectionalLightMaker::Start(Scene *scene)
{
  std::ostringstream stream;
  stream << "user_directional_light_" << counter++;
  this->msg.id = stream.str();
  this->state = 1;
}

void DirectionalLightMaker::Stop()
{
  this->state = 0;
  Events::moveModeSignal(true);
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

  this->msg.pose.pos = event.camera->GetWorldPointOnPlane(event.pressPos.x, event.pressPos.y, norm, 0);
  this->msg.pose.pos.z = 5;
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
  Simulator::Instance()->SendMessage(this->msg);
}
