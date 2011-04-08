/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <wx/sizer.h>
#include <wx/evtloop.h>

#include "common/Messages.hh"

#include "gui/TimePanel.hh"

using namespace gazebo;
using namespace gui;


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
  this->statusUpdatePeriod = 0.5;

  this->statsSub = transport::subscribe("/gazebo/default/world_stats", &TimePanel::OnStats, this);
}

void TimePanel::OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &msg)
{
  this->simTime  = common::Message::Convert( msg->sim_time() );
  this->realTime = common::Message::Convert( msg->real_time() );
  this->pauseTime = common::Message::Convert( msg->pause_time() );
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
TimePanel::~TimePanel()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the time panel
void TimePanel::MyUpdate( )
{
  common::Time percent;

  percent = ( this->simTime / this->realTime);

  wxString simSuffix;
  wxString realSuffix;

  double simDbl = this->simTime.Double();
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

  double realDbl = this->realTime.Double();
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

  str.Printf(wxT("%f sec"), this->pauseTime.Double());
  this->pauseTimeCtrl->SetValue(str);
}
