#include <iostream>
#include <FL/Fl.H>

#include "Events.hh"
#include "MouseEvent.hh"
#include "Simulator.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "World.hh"
#include "SphereMaker.hh"

using namespace gazebo;

SphereMaker::SphereMaker() 
: EntityMaker()
{
  this->state = 0;
  this->visualName = "";
  this->index = 0;
}

SphereMaker::~SphereMaker()
{
}

void SphereMaker::Start()
{
  std::ostringstream stream;
  std::string name = "user_sphere";

  do
  {
    stream.str("");
    stream << name << index;
    this->index++;
  } while (OgreCreator::Instance()->GetVisual(stream.str()));

  this->visualName = stream.str();
  this->state = 1;
}

void SphereMaker::Stop()
{
  OgreVisual *vis = OgreCreator::Instance()->GetVisual(this->visualName);
  if (vis)
    OgreCreator::Instance()->DeleteVisual(this->visualName);

  Events::moveModeSignal(true);
  this->state = 0;
}

bool SphereMaker::IsActive() const
{
  return this->state > 0;
}

void SphereMaker::MousePushCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->mousePushPos = event.pressPos;
}

void SphereMaker::MouseReleaseCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->state++;

  if (this->state == 2)
  {
    this->CreateTheEntity();
    this->Stop();
  }
}

void SphereMaker::MouseDragCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  Vector3 norm;
  Vector3 p1, p2;

  norm.Set(0,0,1);

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
    vis->AttachMesh("unit_sphere");
    vis->SetPosition(p1);
  }

  double scale = p1.Distance(p2);
  Vector3 p = vis->GetPosition();

  p.z = scale;

  vis->SetPosition(p);
  vis->SetScale(Vector3(scale,scale,scale));
}

void SphereMaker::CreateTheEntity()
{
  boost::recursive_mutex::scoped_lock lock( *Simulator::Instance()->GetMRMutex());
  std::ostringstream newModelStr;

  OgreVisual *vis = OgreCreator::Instance()->GetVisual(this->visualName);
  if (!vis)
    return;

  newModelStr << "<?xml version='1.0'?> <gazebo:world xmlns:xi='http://www.w3.org/2001/XInclude' xmlns:gazebo='http://playerstage.sourceforge.net/gazebo/xmlschema/#gz' xmlns:model='http://playerstage.sourceforge.net/gazebo/xmlschema/#model' xmlns:sensor='http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor' xmlns:body='http://playerstage.sourceforge.net/gazebo/xmlschema/#body' xmlns:geom='http://playerstage.sourceforge.net/gazebo/xmlschema/#geom' xmlns:joint='http://playerstage.sourceforge.net/gazebo/xmlschema/#joint' xmlns:interface='http://playerstage.sourceforge.net/gazebo/xmlschema/#interface' xmlns:rendering='http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering' xmlns:renderable='http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable' xmlns:controller='http://playerstage.sourceforge.net/gazebo/xmlschema/#controller' xmlns:physics='http://playerstage.sourceforge.net/gazebo/xmlschema/#physics' >";


  newModelStr << "<model:physical name=\"" << this->visualName << "\">\
    <xyz>" << vis->GetPosition() << "</xyz>\
    <body:sphere name=\"body\">\
    <geom:sphere name=\"geom\">\
    <size>" << vis->GetScale().x << "</size>\
    <mass>0.5</mass>\
    <visual>\
    <mesh>unit_sphere</mesh>\
    <size>" << vis->GetScale()*2 << "</size>\
    <material>Gazebo/Grey</material>\
    <shader>pixel</shader>\
    </visual>\
    </geom:sphere>\
    </body:sphere>\
    </model:physical>";

  newModelStr <<  "</gazebo:world>";

  World::Instance()->InsertEntity(newModelStr.str());

  OgreCreator::Instance()->DeleteVisual(this->visualName);
}
