#include <iostream>
#include <FL/Fl.H>

#include "Events.hh"
#include "MouseEvent.hh"
#include "Simulator.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "CameraManager.hh"
#include "OgreCamera.hh"
#include "World.hh"
#include "BoxMaker.hh"

using namespace gazebo;

BoxMaker::BoxMaker() 
: EntityMaker()
{
  this->state = 0;
  this->visualName = "";
  this->index = 0;
}

BoxMaker::~BoxMaker()
{
}

void BoxMaker::Start()
{
  std::ostringstream stream;
  std::string boxName = "user_box";

  do
  {
    stream.str("");
    stream << boxName << index;
    this->index++;
  } while (OgreCreator::Instance()->GetVisual(stream.str()));

  this->visualName = stream.str();
  this->state = 1;
}

void BoxMaker::Stop()
{
  OgreVisual *vis = OgreCreator::Instance()->GetVisual(this->visualName);
  if (vis)
    OgreCreator::Instance()->DeleteVisual(this->visualName);

  this->state = 0;
  Events::moveModeSignal(true);
}

bool BoxMaker::IsActive() const
{
  return this->state > 0;
}

void BoxMaker::MousePushCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->mousePushPos = event.pressPos;
}

void BoxMaker::MouseReleaseCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->state++;

  if (this->state == 3)
  {
    this->CreateTheEntity();
    this->Stop();
  }
}

void BoxMaker::MouseDragCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  Vector3 norm(0,0,1);
  Vector3 p1, p2;

  p1 = this->GetWorldPointOnPlane(this->mousePushPos.x, this->mousePushPos.y, norm, 0);
  p1 = this->GetSnappedPoint( p1 );

  p2 = this->GetWorldPointOnPlane(event.pos.x, event.pos.y ,norm, 0);
  p2 = this->GetSnappedPoint( p2 );

  OgreVisual *vis = NULL;
  if (OgreCreator::Instance()->GetVisual(this->visualName))
    vis = OgreCreator::Instance()->GetVisual(this->visualName);
  else
  {
    vis = OgreCreator::Instance()->CreateVisual(this->visualName);
    vis->AttachMesh("unit_box_U1V1");
    vis->SetPosition(p1);
  }

  Vector3 scale = p1-p2;
  Vector3 p = vis->GetPosition();

  if (this->state == 1)
  {
    scale.z = 0.01;
    p.x = p1.x - scale.x/2.0;
    p.y = p1.y - scale.y/2.0;
  }
  else
  {
    scale = vis->GetScale();
    scale.z = (this->mousePushPos.y - event.pos.y)*0.01;
    p.z = scale.z/2.0;
  }

  vis->SetPosition(p);

  vis->SetScale(scale);

}

void BoxMaker::CreateTheEntity()
{
  boost::recursive_mutex::scoped_lock lock( *Simulator::Instance()->GetMRMutex());

  std::ostringstream newModelStr;

  OgreVisual *vis = OgreCreator::Instance()->GetVisual(this->visualName);
  if (!vis)
    return;

  newModelStr << "<?xml version='1.0'?> <gazebo:world xmlns:xi='http://www.w3.org/2001/XInclude' xmlns:gazebo='http://playerstage.sourceforge.net/gazebo/xmlschema/#gz' xmlns:model='http://playerstage.sourceforge.net/gazebo/xmlschema/#model' xmlns:sensor='http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor' xmlns:body='http://playerstage.sourceforge.net/gazebo/xmlschema/#body' xmlns:geom='http://playerstage.sourceforge.net/gazebo/xmlschema/#geom' xmlns:joint='http://playerstage.sourceforge.net/gazebo/xmlschema/#joint' xmlns:interface='http://playerstage.sourceforge.net/gazebo/xmlschema/#interface' xmlns:rendering='http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering' xmlns:renderable='http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable' xmlns:controller='http://playerstage.sourceforge.net/gazebo/xmlschema/#controller' xmlns:physics='http://playerstage.sourceforge.net/gazebo/xmlschema/#physics' >";


  newModelStr << "<model:physical name=\"" << this->visualName << "\">\
    <xyz>" << vis->GetPosition() << "</xyz>\
    <body:box name=\"body\">\
    <geom:box name=\"geom\">\
    <size>" << vis->GetScale() << "</size>\
    <mass>0.5</mass>\
    <visual>\
    <mesh>unit_box_U1V1</mesh>\
    <size>" << vis->GetScale() << "</size>\
    <material>Gazebo/Grey</material>\
    <shader>pixel</shader>\
    </visual>\
    </geom:box>\
    </body:box>\
    </model:physical>";

  newModelStr <<  "</gazebo:world>";

  OgreCreator::Instance()->DeleteVisual(this->visualName);

  World::Instance()->InsertEntity(newModelStr.str());
}

