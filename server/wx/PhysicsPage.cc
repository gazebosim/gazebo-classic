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
  
  this->propGrid = new wxPropertyGrid( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxPG_DEFAULT_STYLE|wxPG_SPLITTER_AUTO_CENTER);
  this->propGrid->Connect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( PhysicsPage::OnPropertyChanged), NULL, this);

  unsigned int paramCount = engine->GetParamCount();

  PropertyFactory::RegisterAll();

  for (unsigned int i = 0; i < paramCount; i++)
  {
    Param *param = engine->GetParam(i);
    Property *prop = PropertyFactory::CreateProperty(param, this->propGrid);

    /*wxString paramName = wxString::FromAscii(param->GetKey().c_str());
    wxString paramValue = wxString::FromAscii(param->GetAsString().c_str());

    wxPGProperty *prop = NULL;
    if (param->IsInt())
      prop = this->propGrid->Append(new wxIntProperty(paramName, wxPG_LABEL));
    else if (param->IsUInt())
      prop = this->propGrid->Append(new wxUIntProperty(paramName, wxPG_LABEL));
    else if (param->IsBool())
    {
      prop = this->propGrid->Append(new wxBoolProperty(paramName, wxPG_LABEL));
      if (param->GetAsString() == "1")
        paramValue = wxT("True");
      else 
        paramValue = wxT("False");
    }
    else if (param->IsFloat() || param->IsDouble())
      prop = this->propGrid->Append(new wxFloatProperty(paramName, wxPG_LABEL));
    else if (param->IsStr())
      prop = this->propGrid->Append(new wxStringProperty(paramName, wxPG_LABEL));

    if (prop)
    {
      prop->SetValueFromString( paramValue );
      prop->SetClientData( param );
    }
    */
  }

  boxSizer->Add(this->propGrid, 1, wxEXPAND | wxALL,4);
  this->SetSizer(boxSizer);
  this->Layout();

}

////////////////////////////////////////////////////////////////////////////////
PhysicsPage::~PhysicsPage()
{
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
