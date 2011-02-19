#include "Body.hh"
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
  }
  else if (this->body2->GetMass().GetAsDouble() == 0)
  {
    delBody = this->body2;
    keepBody = this->body1;
  }
  else
    gzerr(0) << "FixedJoint must have one body that has zero mass\n";

  delBody->RemoveFromPhysics();
  while (delBody->GetGeomCount() > 0)
  {
    Geom *g = delBody->GetGeom(0);
    Pose3d geom_pose = g->GetRelativePose();
    geom_pose = geom_pose + delBody->GetRelativePose();
    geom_pose = geom_pose - keepBody->GetRelativePose();

    keepBody->AttachGeom( g );
    delBody->DettachGeom( g );

    delBody->GetCoMEntity()->RemoveChild(g);
    
    keepBody->GetCoMEntity()->AddChild(g);

    g->SetParent(keepBody->GetCoMEntity());
    g->SetRelativePose( geom_pose );
  }
  delete delBody;
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


