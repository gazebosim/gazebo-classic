#include "SimulationFrame.hh"
#include "SimulationApp.hh"
#include "Simulator.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
SimulationApp::SimulationApp()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Load the simulation app
void SimulationApp::Load()
{
  int local_argc = 0;
  char **local_argv = new char*[local_argc];
  for (int i=0; i < local_argc; i++)
    local_argv[i] = strdup(wxString(argv[i]).mb_str());

  wxEntryStart(local_argc, local_argv);
  wxTheApp->OnInit();
}

////////////////////////////////////////////////////////////////////////////////
// Run the gui
void SimulationApp::Run()
{
  wxTheApp->OnRun();
}

////////////////////////////////////////////////////////////////////////////////
// Init the simulation app
void SimulationApp::Init()
{
  this->frame->Init();
}

////////////////////////////////////////////////////////////////////////////////
/// On Init
bool SimulationApp::OnInit()
{
  this->frame = new SimulationFrame(NULL);
  this->frame->Show();
  this->SetTopWindow(this->frame);

  //Connect( wxID_ANY, wxEVT_IDLE, wxIdleEventHandler(SimulationApp::OnIdle) );
  Connect( this->timer.GetId(), wxEVT_TIMER, wxTimerEventHandler(SimulationApp::OnIdle), NULL, this );
  this->timer.Start(33);

  this->frame->CreateCameras();
  return true;
}

void SimulationApp::OnIdle(wxTimerEvent &evt)
{
  Simulator::Instance()->GraphicsUpdate();
}

////////////////////////////////////////////////////////////////////////////////
/// Initalize the gui
void SimulationApp::Update()
{
  this->frame->Update();
}

////////////////////////////////////////////////////////////////////////////////
/// Save the gui params in xml format
void SimulationApp::Save(std::string &prefix, std::ostream &stream)
{
}
