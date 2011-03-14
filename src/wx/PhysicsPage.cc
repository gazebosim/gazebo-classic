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
#include "propgrid/propgrid.h"

#include "Properties.hh"
#include "PhysicsEngine.hh"
#include "common/Param.hh"
#include "World.hh"
#include "Simulator.hh"
#include "PhysicsPage.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
PhysicsPage::PhysicsPage(wxWindow *parent)
  : ParamPage( parent )
{
  this->SetName(wxT("physics"));

  PhysicsEngine *engine = Simulator::Instance()->GetActiveWorld()->GetPhysicsEngine();

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
