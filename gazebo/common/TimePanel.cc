#include <wx/sizer.h>
#include <wx/evtloop.h>

#include "Simulator.hh"
#include "TimePanel.hh"

using namespace gazebo;


////////////////////////////////////////////////////////////////////////////////
// Constructor
TimePanel::TimePanel( wxWindow *parent )
  : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxSize(200,40), wxTAB_TRAVERSAL)
{
  wxBoxSizer *boxSizer1 = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer *boxSizer2 = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer *boxSizer3 = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer *boxSizer4 = new wxBoxSizer(wxHORIZONTAL);
  wxBoxSizer *boxSizer5 = new wxBoxSizer(wxHORIZONTAL);

  this->percentRealTimeText = new wxStaticText( this, wxID_ANY, wxT("Real Time Factor:"), wxDefaultPosition, wxDefaultSize, 0);
  this->percentRealTimeText->Wrap(-1);
  boxSizer2->Add(this->percentRealTimeText, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

  this->percentRealTimeCtrl = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
  boxSizer2->Add(this->percentRealTimeCtrl, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5);

  boxSizer1->Add(boxSizer2, 1, wxEXPAND, 5);


  this->simTimeText = new wxStaticText( this, wxID_ANY, wxT("Sim Time:"), wxDefaultPosition, wxDefaultSize, 0);
  this->simTimeText->Wrap(-1);
  boxSizer3->Add(this->simTimeText, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

  this->simTimeCtrl = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
  boxSizer3->Add(this->simTimeCtrl, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5);

  boxSizer1->Add(boxSizer3, 1, wxEXPAND, 5);


  this->realTimeText = new wxStaticText( this, wxID_ANY, wxT("Real Time:"), wxDefaultPosition, wxDefaultSize, 0);
  this->realTimeText->Wrap(-1);
  boxSizer4->Add(this->realTimeText, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

  this->realTimeCtrl = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
  boxSizer4->Add(this->realTimeCtrl, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5);

  boxSizer1->Add(boxSizer4, 1, wxEXPAND, 5);


  this->pauseTimeText = new wxStaticText( this, wxID_ANY, wxT("Pause Time:"), wxDefaultPosition, wxDefaultSize, 0);
  this->pauseTimeText->Wrap(-1);
  boxSizer5->Add(this->pauseTimeText, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

  this->pauseTimeCtrl = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
  boxSizer5->Add(this->pauseTimeCtrl, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5);

  boxSizer1->Add(boxSizer5, 1, wxEXPAND, 5);

  this->SetSizer(boxSizer1);
  this->Layout();

  this->lastUpdateTime = 0;
  this->statusUpdatePeriod = 0.05;
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
TimePanel::~TimePanel()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the time panel
void TimePanel::Update()
{
  //while (wxEventLoop::GetActive()->Pending())
    //wxEventLoop::GetActive()->Dispatch();

  if (Simulator::Instance()->GetRealTime() - this->lastUpdateTime > this->statusUpdatePeriod)
  {
    Time simTime = Simulator::Instance()->GetSimTime();
    Time realTime = Simulator::Instance()->GetRealTime();
    Time percent;
    
    if (realTime < this->statusUpdatePeriod )
    {
      percent = ( simTime / realTime);
      this->percentLastRealTime =0;
      this->percentLastSimTime = 0;
    }
    else
    {
      percent = ((simTime - this->percentLastSimTime) / 
                 (realTime - this->percentLastRealTime)).Double();

      this->percentLastRealTime = realTime;
      this->percentLastSimTime = simTime;
    }

    wxString simSuffix;
    wxString realSuffix;

    double simDbl = simTime.Double();
    if (simDbl > 31536000)
      simSuffix << simDbl/31536000 << wxT("\tdys");
    else if (simDbl > 86400)
      simSuffix << simDbl / 86400 << wxT("\tdys");
    else if (simDbl > 3600)
      simSuffix << simDbl/3600 << wxT("\thrs");
    else if (simDbl > 999)
      simSuffix << simDbl/60 << wxT("\tmin");
    else
      simSuffix << simDbl << wxT("\tsec");

    double realDbl = realTime.Double();
    if (realDbl > 31536000)
      realSuffix << realDbl/31536000 << wxT("\tdys");
    else if (realDbl > 86400)
      realSuffix << realDbl/86400 << wxT("\tdys");
    else if (realDbl > 3600)
      realSuffix << realDbl/3600 << wxT("\thrs");
    else if (realDbl > 999)
      realSuffix << realDbl/60 << wxT("\tmin");
    else
      realSuffix << realDbl << wxT("\tsec");

    wxString str;

    str.Printf(wxT("%f"), percent.Double());
    this->percentRealTimeCtrl->SetValue( str );

    this->simTimeCtrl->SetValue(simSuffix);
    this->realTimeCtrl->SetValue(realSuffix);

    str.Printf(wxT("%f sec"), Simulator::Instance()->GetPauseTime().Double());
    this->pauseTimeCtrl->SetValue(str);

    this->lastUpdateTime = Simulator::Instance()->GetRealTime();
  }

}
