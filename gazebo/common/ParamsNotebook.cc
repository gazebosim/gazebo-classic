#include <wx/notebook.h>

#include "GridPage.hh"
#include "FogPage.hh"
#include "PhysicsPage.hh"
#include "RenderingPage.hh"
#include "ParamsNotebook.hh"

using namespace gazebo;

ParamsNotebook::ParamsNotebook( wxWindow *parent )
  : wxDialog(parent, wxID_ANY, wxT("Params") )
{
  wxBoxSizer *boxSizer = new wxBoxSizer(wxVERTICAL);

  this->notebook = new wxNotebook(this, wxID_ANY);
  this->notebook->AddPage( new PhysicsPage(this->notebook), wxT("Physics") );
  this->notebook->AddPage( new RenderingPage(this->notebook), wxT("Rendering"));
  this->notebook->AddPage( new GridPage(this->notebook), wxT("Grid") );
  this->notebook->AddPage( new FogPage(this->notebook), wxT("Fog") );

  boxSizer->Add(this->notebook,1,wxEXPAND, 0);

  wxBoxSizer *buttonSizer = new wxBoxSizer(wxHORIZONTAL);
  wxButton *applyButton = new wxButton( this, wxID_ANY, wxT("Apply") );
  applyButton->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(ParamsNotebook::OnApply), NULL, this);
  buttonSizer->Add(applyButton);

  wxButton *doneButton = new wxButton( this, wxID_ANY, wxT("Done") );
  doneButton->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(ParamsNotebook::OnDone), NULL, this);
  buttonSizer->Add(doneButton,0, wxRIGHT, 2);


  boxSizer->Add(buttonSizer, 0, wxALL, 2);

  this->SetSizer(boxSizer);
  boxSizer->Fit(this);

  this->Layout();
}

ParamsNotebook::~ParamsNotebook()
{
}

////////////////////////////////////////////////////////////////////////////////
// On done event
void ParamsNotebook::OnDone(wxCommandEvent &event)
{
  this->OnApply(event);
  this->EndModal(wxID_OK);
}

////////////////////////////////////////////////////////////////////////////////
// On apply event
void ParamsNotebook::OnApply(wxCommandEvent &event)
{
  wxWindow *page = this->notebook->GetPage( this->notebook->GetSelection() );
  std::string pageName = std::string(page->GetName().mb_str());

  if (pageName == "grid")
    ((GridPage*)page)->Apply();
  else if (pageName == "fog")
    ((FogPage*)page)->Apply();
}
