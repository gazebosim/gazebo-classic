#include <boost/lexical_cast.hpp>

#include "RenderControl.hh"
#include "UserCamera.hh"
#include "Pose3d.hh"
#include "Global.hh"
#include "RenderPanel.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
RenderPanel::RenderPanel(wxWindow *parent)
  : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxSize(320,240), wxTAB_TRAVERSAL)
{
  wxBoxSizer *boxSizer1 = new wxBoxSizer(wxVERTICAL);
  wxBoxSizer *boxSizer2 = new wxBoxSizer(wxHORIZONTAL);

  this->renderControl = new RenderControl(this);
  boxSizer1->Add(this->renderControl, 2, wxALL | wxEXPAND );

  wxStaticText *xyzPosText = new wxStaticText( this, wxID_ANY, wxT("XYZ:"), wxDefaultPosition, wxDefaultSize, 0);
  xyzPosText->Wrap(-1);
  boxSizer2->Add(xyzPosText, 0, wxLEFT | wxALIGN_CENTER_VERTICAL, 5);

  this->xPosCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxTextValidator(wxFILTER_NUMERIC));
  this->xPosCtrl->Connect(wxEVT_SET_FOCUS, wxFocusEventHandler(RenderPanel::OnXPosSetFocus), NULL, this);
  this->xPosCtrl->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler(RenderPanel::OnXPosKillFocus), NULL, this);
  boxSizer2->Add(this->xPosCtrl, 0, wxALL | wxALIGN_CENTER_VERTICAL, 1);

  this->yPosCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize,0, wxTextValidator(wxFILTER_NUMERIC));
  this->yPosCtrl->Connect(wxEVT_SET_FOCUS, wxFocusEventHandler(RenderPanel::OnYPosSetFocus), NULL, this);
  this->yPosCtrl->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler(RenderPanel::OnYPosKillFocus), NULL, this);
  boxSizer2->Add(this->yPosCtrl, 0, wxALL | wxALIGN_CENTER_VERTICAL, 1);

  this->zPosCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize,0, wxTextValidator(wxFILTER_NUMERIC));
  this->zPosCtrl->Connect(wxEVT_SET_FOCUS, wxFocusEventHandler(RenderPanel::OnZPosSetFocus), NULL, this);
  this->zPosCtrl->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler(RenderPanel::OnZPosKillFocus), NULL, this);
  boxSizer2->Add(this->zPosCtrl, 0, wxRIGHT | wxALIGN_CENTER_VERTICAL, 5);



  wxStaticText *rpyText = new wxStaticText( this, wxID_ANY, wxT("RPY:"), wxDefaultPosition, wxDefaultSize, 0);
  rpyText->Wrap(-1);
  boxSizer2->Add(rpyText, 0, wxALL | wxALIGN_CENTER_VERTICAL, 1);

  this->rollCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxTextValidator(wxFILTER_NUMERIC));
  this->rollCtrl->Connect(wxEVT_SET_FOCUS, wxFocusEventHandler(RenderPanel::OnRollSetFocus), NULL, this);
  this->rollCtrl->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler(RenderPanel::OnRollKillFocus), NULL, this);
  boxSizer2->Add(this->rollCtrl, 0, wxALL | wxALIGN_CENTER_VERTICAL, 1);

  this->pitchCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize,0, wxTextValidator(wxFILTER_NUMERIC) );
  this->pitchCtrl->Connect(wxEVT_SET_FOCUS, wxFocusEventHandler(RenderPanel::OnPitchSetFocus), NULL, this);
  this->pitchCtrl->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler(RenderPanel::OnPitchKillFocus), NULL, this);
  boxSizer2->Add(this->pitchCtrl, 0, wxALL | wxALIGN_CENTER_VERTICAL, 1);

  this->yawCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize,0, wxTextValidator(wxFILTER_NUMERIC));
  this->yawCtrl->Connect(wxEVT_SET_FOCUS, wxFocusEventHandler(RenderPanel::OnYawSetFocus), NULL, this);
  this->yawCtrl->Connect(wxEVT_KILL_FOCUS, wxFocusEventHandler(RenderPanel::OnYawKillFocus), NULL, this);
  boxSizer2->Add(this->yawCtrl, 0, wxRIGHT | wxALIGN_CENTER_VERTICAL, 5);



  wxStaticText *fpsText = new wxStaticText( this, wxID_ANY, wxT("FPS:"), wxDefaultPosition, wxDefaultSize, 0);
  fpsText->Wrap(-1);
  boxSizer2->Add(fpsText, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

  this->fpsCtrl = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
  boxSizer2->Add(this->fpsCtrl, 0, wxALL | wxALIGN_CENTER_VERTICAL, 1);


  boxSizer1->Add(boxSizer2,0,wxEXPAND,5);

  this->SetSizer(boxSizer1);
  this->Layout();

  this->xUpdate = true;
  this->yUpdate = true;
  this->zUpdate = true;
  this->rollUpdate = true;
  this->pitchUpdate = true;
  this->yawUpdate = true;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RenderPanel::~RenderPanel()
{
  delete this->renderControl;
}

////////////////////////////////////////////////////////////////////////////////
// Init
void RenderPanel::Init()
{
  this->renderControl->Init();
}

////////////////////////////////////////////////////////////////////////////////
// Create a camera
void RenderPanel::CreateCamera(unsigned int sceneMgr)
{
  this->renderControl->CreateCamera(sceneMgr);
}

////////////////////////////////////////////////////////////////////////////////
// Update the render panel
void RenderPanel::Update()
{
  UserCamera *cam = this->renderControl->GetCamera();
  if (!cam)
    return;

  float fps = cam->GetAvgFPS();
  Pose3d pose = cam->GetWorldPose();

  wxString str;

  if (this->xUpdate)
  {
    str.Printf(wxT("%6.2f"), pose.pos.x);
    this->xPosCtrl->ChangeValue(str);
  }

  if (this->yUpdate)
  {
    str.Printf(wxT("%6.2f"), pose.pos.y);
    this->yPosCtrl->SetValue(str);
  }

  if (this->zUpdate)
  {
    str.Printf(wxT("%6.2f"), pose.pos.z);
    this->zPosCtrl->SetValue(str);
  }

  if (this->rollUpdate)
  {
    str.Printf(wxT("%6.2f"), RTOD(pose.rot.GetAsEuler().x));
    this->rollCtrl->SetValue(str);
  }

  if (this->pitchUpdate)
  {
    str.Printf(wxT("%6.2f"), RTOD(pose.rot.GetAsEuler().y));
    this->pitchCtrl->SetValue(str);
  }

  if (this->yawUpdate)
  {
    str.Printf(wxT("%6.2f"), RTOD(pose.rot.GetAsEuler().z));
    this->yawCtrl->SetValue(str);
  }

  str.Printf(wxT("%6.2f"), fps);
  this->fpsCtrl->SetValue(str);
}

void RenderPanel::OnXPosSetFocus(wxFocusEvent &event)
{
  this->xUpdate = false;
}

void RenderPanel::OnYPosSetFocus(wxFocusEvent &event)
{
  this->yUpdate = false;
}

void RenderPanel::OnZPosSetFocus(wxFocusEvent &event)
{
  this->zUpdate = false;
}

void RenderPanel::OnRollSetFocus(wxFocusEvent &event)
{
  this->rollUpdate = false;
}

void RenderPanel::OnPitchSetFocus(wxFocusEvent &event)
{
  this->pitchUpdate = false;
}

void RenderPanel::OnYawSetFocus(wxFocusEvent &event)
{
  this->yawUpdate = false;
}




void RenderPanel::OnXPosKillFocus(wxFocusEvent &event)
{
  UserCamera *cam = this->renderControl->GetCamera();
  Vector3 pos = cam->GetWorldPosition();

  std::string str = std::string(this->xPosCtrl->GetValue().mb_str());
  boost::trim(str);

  pos.x = boost::lexical_cast<float>(str);

  cam->SetWorldPosition(pos);

  this->xUpdate = true;
}

void RenderPanel::OnYPosKillFocus(wxFocusEvent &event)
{
  UserCamera *cam = this->renderControl->GetCamera();
  Vector3 pos = cam->GetWorldPosition();

  std::string str = std::string(this->yPosCtrl->GetValue().mb_str());
  boost::trim(str);

  pos.y = boost::lexical_cast<float>(str);

  cam->SetWorldPosition(pos);

  this->yUpdate = true;
}

void RenderPanel::OnZPosKillFocus(wxFocusEvent &event)
{
  UserCamera *cam = this->renderControl->GetCamera();
  Vector3 pos = cam->GetWorldPosition();

  std::string str = std::string(this->zPosCtrl->GetValue().mb_str());
  boost::trim(str);

  pos.z = boost::lexical_cast<float>(str);

  cam->SetWorldPosition(pos);

  this->zUpdate = true;
}

void RenderPanel::OnRollKillFocus(wxFocusEvent &event)
{
  UserCamera *cam = this->renderControl->GetCamera();
  Pose3d pose = cam->GetWorldPose();
  Vector3 rpy = pose.rot.GetAsEuler();

  std::string str = std::string(this->rollCtrl->GetValue().mb_str());
  boost::trim(str);

  rpy.x = DTOR(boost::lexical_cast<float>(str));

  pose.rot.SetFromEuler(rpy);
  cam->SetWorldPose(pose);

  this->rollUpdate = true;
}

void RenderPanel::OnPitchKillFocus(wxFocusEvent &event)
{
  UserCamera *cam = this->renderControl->GetCamera();
  Pose3d pose = cam->GetWorldPose();
  Vector3 rpy = pose.rot.GetAsEuler();

  std::string str = std::string(this->pitchCtrl->GetValue().mb_str());
  boost::trim(str);

  rpy.y = DTOR(boost::lexical_cast<float>(str));

  pose.rot.SetFromEuler(rpy);
  cam->SetWorldPose(pose);

  this->pitchUpdate = true;
}

void RenderPanel::OnYawKillFocus(wxFocusEvent &event)
{
  UserCamera *cam = this->renderControl->GetCamera();
  Pose3d pose = cam->GetWorldPose();
  Vector3 rpy = pose.rot.GetAsEuler();

  std::string str = std::string(this->yawCtrl->GetValue().mb_str());
  boost::trim(str);

  rpy.z = DTOR(boost::lexical_cast<float>(str));

  pose.rot.SetFromEuler(rpy);
  cam->SetWorldPose(pose);

  this->yawUpdate = true;
}
