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

Joint *FixedJoint::IsConnected(Body *b)
{
  for (unsigned int i=0; i < this->model->GetJointCount(); i++)
  {
    Joint *j = this->model->GetJoint(i);
    if (j->GetType() == Joint::FIXED)
      continue;
    if (j->body1 == b || j->body2 == b)
      return j;
  }

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Load joint
void FixedJoint::Load(XMLConfigNode *node)
{
  Joint::Load(node);
  Joint *prevJoint = NULL;
  Body *delBody = NULL;
  Body *keepBody = NULL;

  std::cout << "-----------------\n";
  std::cout << "FIXED JOINT[" << this->GetName() << "]\n";

  if (this->IsConnected(this->body1) && !this->IsConnected(this->body2))
  {
    prevJoint = this->IsConnected(this->body1);
    std::cout << "  Body1 is connected\n";
    delBody = this->body2;
    keepBody = this->body1;
    this->body2 = NULL;
  }
  if (this->IsConnected(this->body2) && !this->IsConnected(this->body1))
  {
    prevJoint = this->IsConnected(this->body2);
    std::cout << "  Body2 is connected\n";
    delBody = this->body1;
    keepBody = this->body2;
    this->body1 = NULL;
  }
  else
  {
    std::cout << "  Default to Body2\n";
    delBody = this->body2;
    keepBody = this->body1;
    this->body2 = NULL;
  }

  this->subsumedOffset = delBody->GetRelativePose() - keepBody->GetRelativePose();

  Pose3d origCoM = keepBody->GetCoMEntity()->GetRelativePose();

    std::cout << "Del Body[" << delBody->GetName() << "] RelPos[" << delBody->GetRelativePose().pos << "] COM[" << delBody->GetCoMEntity()->GetRelativePose().pos << "]\n";
    std::cout << "  Mass[" << delBody->GetMass() << "]\n";
    std::cout << "Keep Body[" << keepBody->GetName() << "] RelPos[" << keepBody->GetRelativePose().pos << "] COM[" << keepBody->GetCoMEntity()->GetRelativePose() << "]\n";
    std::cout << "  Mass[" << keepBody->GetMass() << "]\n";
  delBody->RemoveFromPhysics();

  /*std::cout << "*******\n";
  for (unsigned int i=0; i < this->model->GetJointCount(); i++)
  {
    Joint *j = this->model->GetJoint(i);
    if (j->GetType() == Joint::FIXED)
      continue;
    Body *b1 = j->body1;
    Body *b2 = j->body2;
    std::cout << "Joint[" << j->GetName() << "] B1[" << b1->GetName() << "] B2[" << b2->GetName() << "]\n";
    Vector3 origODEAnchor = j->GetAnchor(0);
    Vector3 origAnchor = j->anchorPos;

    if (**(j->anchorBodyNameP) == delBody->GetName())
    {
      std::cout << "Switching Anchor Body\n";
      j->anchorBodyNameP->SetValue(keepBody->GetName());
      j->anchorBody = keepBody;
    }

    Vector3 na = j->anchorBody->GetWorldPose().pos;// - (keepBody->GetRelativePose() - delBody->GetRelativePose()).pos;
    std::cout << "Orig Anchor[" << origAnchor << "] ODE Anchor[" << origODEAnchor << "] NewA[" << na << "]\n";

    if (b1->GetName() == delBody->GetName())
    {
      std::cout << "Body0\n";
      j->Attach(keepBody, b2);
  //    j->SetAnchor( 0, na);
      std::cout << " New 0[" << j->GetJointBody(0)->GetName() << "] 1[" << j->GetJointBody(1)->GetName() << "]\n";
    }
    else if (b2->GetName() == delBody->GetName())
    {
      std::cout << "Body1\n";
      j->Attach(b1, keepBody);
   //   j->SetAnchor( 0, j->anchorPos);

      //j->SetAnchor( 0, j->GetAnchor(0) + (delBody->GetRelativePose() - keepBody->GetRelativePose()).pos);
      std::cout << " New 0[" << j->GetJointBody(0)->GetName() << "] 1[" << j->GetJointBody(1)->GetName() << "]\n";
    }
    else
      std::cout << "Neither\n";
  }
  std::cout << "Done\n";
  */
  


  while (delBody->GetGeomCount() > 0)
  {
    Geom *g = delBody->GetGeom(0);
    Pose3d geom_pose = g->GetRelativePose() + delBody->GetCoMEntity()->GetRelativePose();

    std::cout << "Geom[" << g->GetName() << "] Pose[" << geom_pose.pos << "]\n";
    std::cout << "  Adding[" << delBody->GetRelativePose().pos << "]\n";

    geom_pose = geom_pose + delBody->GetRelativePose();
    std::cout << "Geom PoseB[" << geom_pose.pos << "] \n";

    std::cout << "  Subtracting[" << keepBody->GetRelativePose().pos << "]\n";
    geom_pose = geom_pose - (keepBody->GetRelativePose() + keepBody->GetCoMEntity()->GetRelativePose());
    std::cout << "Geom PoseC[" << geom_pose.pos << "]\n";


    keepBody->AttachGeom( g );
    delBody->DettachGeom( g );

    delBody->GetCoMEntity()->RemoveChild(g);
    
    g->SetName(delBody->GetName());
    keepBody->GetCoMEntity()->AddChild(g);

    g->SetParent(keepBody->GetCoMEntity());
    g->SetRelativePose( geom_pose );
    g->SetBody(keepBody);
  }

  Mass mass = delBody->GetMass();
  mass.SetCoG(mass.GetCoG() + (delBody->GetRelativePose() - keepBody->GetRelativePose()).pos );
  keepBody->SetMass( keepBody->GetMass() + mass  );
  keepBody->UpdateCoM();
  std::cout << "New Mass[" << keepBody->GetMass() << "]\n";
  std::cout << "Keep Body[" << keepBody->GetName() << "] RelPos[" << keepBody->GetRelativePose().pos << "] COM[" << keepBody->GetCoMEntity()->GetRelativePose() << "]\n";

  if (prevJoint)
  {
    Pose3d diff = keepBody->GetCoMEntity()->GetRelativePose() - origCoM;
    std::cout << "\n DIFF[" << diff << "]\n";
    prevJoint->SetAnchor(0,prevJoint->anchorPos + diff.pos);
  }
  
  keepBody->AddAltName(delBody->GetName());
  keepBody->MergeAltNames(delBody);


  ((Entity*)this->model)->RemoveChild(delBody);
  delete delBody;

  this->line1->setMaterial("Gazebo/RedEmissive");
  this->line2->setMaterial("Gazebo/RedEmissive");
}

////////////////////////////////////////////////////////////////////////////////
/// Update the joint
void FixedJoint::Update()
{
  //this->jointUpdateSignal();

  //TODO: Evaluate impact of this code on performance
  if (this->visual && this->visual->GetVisible())
  {
    Body *anchor = this->model->GetBody(**this->anchorBodyNameP);
    Body *body = this->model->GetBody(**this->body1NameP);

    this->anchorPos = (Pose3d(**(this->anchorOffsetP),Quatern()) + 
        anchor->GetWorldPose()).pos;

    this->visual->SetPosition(this->anchorPos);

    this->line1->SetPoint(1, body->GetWorldPose().pos - this->anchorPos);
    this->line2->SetPoint(1, body->GetWorldPose().rot.RotateVector( this->subsumedOffset.pos) );
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


