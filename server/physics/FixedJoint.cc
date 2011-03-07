#include "Sensor.hh"
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

  if (this->IsConnected(this->body1) && !this->IsConnected(this->body2))
  {
    prevJoint = this->IsConnected(this->body1);
    delBody = this->body2;
    keepBody = this->body1;
    this->body2 = NULL;
  }
  if (this->IsConnected(this->body2) && !this->IsConnected(this->body1))
  {
    prevJoint = this->IsConnected(this->body2);
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


  bool test = false;
  if (delBody->GetName() == "narrow_stereo_gazebo_l_stereo_camera_frame")
  {
    std::cerr << "\n\n DEL Narrow Stereo \n\n";
    test = true;
  }

  std::vector<Sensor*>::iterator iter;
  std::vector<Sensor*> sensors = delBody->GetSensors();
  for (iter = sensors.begin(); iter != sensors.end(); iter++)
  {
    Pose3d p = delBody->GetRelativePose() - keepBody->GetRelativePose();

    if (test)
    {
      std::cout << "Sensor Pose[" << (*iter)->GetRelativePose() << "] Parent[" << (*iter)->GetParent()->GetName() << "] DelBody[" << delBody->GetName() << "] Pose[" << p << "]";
    }

    keepBody->AddSensor(*iter);
    (*iter)->SetBody(keepBody);
    (*iter)->SetParent(keepBody);
    (*iter)->SetRelativePose(p);
    keepBody->AddChild(*iter);
    delBody->RemoveChild(*iter);
  }
  delBody->ClearSensors();

  this->subsumedOffset = delBody->GetRelativePose() - keepBody->GetRelativePose();

  Pose3d origCoM = keepBody->GetCoMEntity()->GetRelativePose();
  delBody->RemoveFromPhysics();
  while (delBody->GetGeomCount() > 0)
  {
    Geom *g = delBody->GetGeom(0);
    Pose3d geom_pose = g->GetRelativePose() + delBody->GetCoMEntity()->GetRelativePose();

    geom_pose = geom_pose + delBody->GetRelativePose();
    geom_pose = geom_pose - (keepBody->GetRelativePose() + keepBody->GetCoMEntity()->GetRelativePose());

    keepBody->AttachGeom( g );
    delBody->DettachGeom( g );

    delBody->GetCoMEntity()->RemoveChild(g);
    
    g->AddAltName(delBody->GetName());
    keepBody->GetCoMEntity()->AddChild(g);

    std::cerr << "Geom[" << g->GetName() << "] Pose[" << geom_pose << "]\n";
    g->SetParent(keepBody->GetCoMEntity());
    g->SetRelativePose( geom_pose );
    g->SetBody(keepBody);
  }

  Mass mass = delBody->GetMass();
  mass.SetCoG(mass.GetCoG() + (delBody->GetRelativePose() - keepBody->GetRelativePose()).pos );
  keepBody->SetMass( keepBody->GetMass() + mass  );
  keepBody->UpdateCoM();

  if (prevJoint)
  {
    Pose3d diff = keepBody->GetCoMEntity()->GetRelativePose() - origCoM;
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


