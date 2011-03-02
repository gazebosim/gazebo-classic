#include "Model.hh"
#include "Body.hh"
#include "OgreVisual.hh"
#include "OgreDynamicLines.hh"
#include "Geom.hh"
#include "OgreAdaptor.hh"
#include "FixedJoint.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
FixedJoint::FixedJoint()
{
  this->type = Joint::FIXED;
  std::cout << "Creating a fixed joint\n";
}

////////////////////////////////////////////////////////////////////////////////
///  Destructor
FixedJoint::~FixedJoint()
{ }

////////////////////////////////////////////////////////////////////////////////
/// Load joint
void FixedJoint::Load(XMLConfigNode *node)
{
  Joint::Load(node);
  Body *delBody = NULL;
  Body *keepBody = NULL;

  if (this->body1->GetMass().GetAsDouble() == 0)
  {
    delBody = this->body1;
    keepBody = this->body2;
    this->body1 = NULL;
  }
  else
  {
    delBody = this->body2;
    keepBody = this->body1;
    this->body2 = NULL;
  }

  this->subsumedOffset = delBody->GetRelativePose() - keepBody->GetRelativePose();

  delBody->RemoveFromPhysics();
  this->model->RemoveChild(delBody);
  while (delBody->GetGeomCount() > 0)
  {
    Geom *g = delBody->GetGeom(0);
    Pose3d geom_pose = g->GetRelativePose();
    geom_pose = geom_pose + delBody->GetRelativePose();
    geom_pose = geom_pose - keepBody->GetRelativePose();

    keepBody->AttachGeom( g );
    delBody->DettachGeom( g );

    delBody->GetCoMEntity()->RemoveChild(g);
    
    g->SetName(delBody->GetName());
    keepBody->GetCoMEntity()->AddChild(g);

    g->SetParent(keepBody->GetCoMEntity());
    g->SetRelativePose( geom_pose );
  }

  keepBody->SetAltName(delBody->GetName());
  delete delBody;

  this->line1->setMaterial("Gazebo/RedEmissive");
  this->line2->setMaterial("Gazebo/RedEmissive");
}

////////////////////////////////////////////////////////////////////////////////
/// Update the joint
void FixedJoint::Update()
{
  this->jointUpdateSignal();

  //TODO: Evaluate impact of this code on performance
  if (this->visual && this->visual->GetVisible())
  {
    this->anchorPos = (Pose3d(**(this->anchorOffsetP),Quatern()) + 
        this->anchorBody->GetWorldPose()).pos;

    this->visual->SetPosition(this->anchorPos);

    this->line1->SetPoint(1, this->body1->GetWorldPose().pos - this->anchorPos);
    this->line2->SetPoint(1, this->body1->GetWorldPose().rot.RotateVector( this->subsumedOffset.pos) );
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Save a joint to a stream in XML format
void FixedJoint::SaveJoint(std::string &prefix, std::ostream &stream)
{
  Joint::SaveJoint(prefix, stream);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the body to which the joint is attached according the _index
Body *FixedJoint::GetJointBody( int index ) const
{ 
  if (index == 1) 
    return this->body1;
  else 
    return this->body2;
}


