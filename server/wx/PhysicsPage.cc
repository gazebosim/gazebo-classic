#include <wx/sizer.h>
#include "propgrid/propgrid.h"

#include "Properties.hh"
#include "PhysicsEngine.hh"
#include "Param.hh"
#include "World.hh"
#include "PhysicsPage.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
PhysicsPage::PhysicsPage(wxWindow *parent)
  : ParamPage( parent )
{
  this->SetName(wxT("physics"));

  PhysicsEngine *engine = World::Instance()->GetPhysicsEngine();

  wxBoxSizer *boxSizer = new wxBoxSizer(wxVERTICAL);

  PropertyFactory::RegisterAll();

  this->propGrid = new wxPropertyGrid( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxPG_DEFAULT_STYLE|wxPG_SPLITTER_AUTO_CENTER);

  this->propGrid->Connect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( PhysicsPage::OnPropertyChanged), NULL, this);

  this->propManager = new PropertyManager(this->propGrid);

  unsigned int paramCount = engine->GetParamCount();

  for (unsigned int i = 0; i < paramCount; i++)
  {
    this->propManager->AddProperty( engine->GetParam(i) ); 
  }

  boxSizer->Add(this->propGrid, 1, wxEXPAND | wxALL,4);
  this->SetSizer(boxSizer);
  this->Layout();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
PhysicsPage::~PhysicsPage()
{
  delete this->propManager;
  this->propGrid->Destroy();
}

////////////////////////////////////////////////////////////////////////////////
void PhysicsPage::Apply()
{
}

////////////////////////////////////////////////////////////////////////////////
// On property changed event callback
void PhysicsPage::OnPropertyChanged(wxPropertyGridEvent &event)
{
  wxPGProperty *wxprop = event.GetProperty();

  if (!wxprop)
    return;

  Property *prop = (Property*)(wxprop->GetClientData());

  if (prop)
    prop->Changed();
}
