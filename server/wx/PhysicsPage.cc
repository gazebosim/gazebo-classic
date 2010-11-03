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
  this->grid->Destroy();
}

////////////////////////////////////////////////////////////////////////////////
void PhysicsPage::Apply()
{
}
