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
#include "propgrid/propgrid.h"

#include "common/Vector3.hh"
#include "common/Quatern.hh"
#include "common/Time.hh"
#include "gui/Properties.hh"

using namespace gazebo;
using namespace gui;


GZ_REGISTER_WX_PROPERTY(typeid(float).name(),        FloatProperty);
GZ_REGISTER_WX_PROPERTY(typeid(double).name(),       DoubleProperty);
GZ_REGISTER_WX_PROPERTY(typeid(int).name(),          IntProperty);
GZ_REGISTER_WX_PROPERTY(typeid(unsigned int).name(), UIntProperty);
GZ_REGISTER_WX_PROPERTY(typeid(bool).name(),         BoolProperty);
GZ_REGISTER_WX_PROPERTY(typeid(std::string).name(),  StringProperty);
GZ_REGISTER_WX_PROPERTY(typeid(common::Vector3).name(), Vector3Property);
GZ_REGISTER_WX_PROPERTY(typeid(common::Quatern).name(),      QuaternProperty);
GZ_REGISTER_WX_PROPERTY(typeid(common::Time).name(),         TimeProperty);

std::map<std::string, PropertyFactoryFn> PropertyFactory::properties; 

void PropertyFactory::RegisterAll()
{
  RegisterFloatProperty();
  RegisterDoubleProperty();
  RegisterIntProperty();
  RegisterUIntProperty();
  RegisterBoolProperty();
  RegisterStringProperty();
  RegisterVector3Property();
  RegisterQuaternProperty();
  RegisterTimeProperty();
}

void PropertyFactory::RegisterProperty(std::string type, 
                                       PropertyFactoryFn factoryfn)
{
  properties[type] = factoryfn;
}

Property *PropertyFactory::CreateProperty(common::Param *p, wxPropertyGrid *g)
{
  std::map<std::string, PropertyFactoryFn>::iterator iter;
  iter = properties.find(p->GetTypename());

  if (iter != properties.end())
    return (iter->second) (p,g);
  else
    std::cerr << "Unable to create property[" << p->GetTypename() << "]\n";

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
PropertyManager::PropertyManager(wxPropertyGrid *g)
  : grid(g)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
PropertyManager::~PropertyManager()
{
  std::list<Property*>::iterator iter;

  for (iter = this->properties.begin(); iter != this->properties.end(); iter++)
    delete *iter;
  this->properties.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Add a property
void PropertyManager::AddProperty(common::Param *p)
{
  Property *prop = PropertyFactory::CreateProperty( p, this->grid );
  this->properties.push_back( prop );
}

////////////////////////////////////////////////////////////////////////////////
// Constructor
Property::Property(common::Param *p, wxPropertyGrid *grid)
  : param(p), grid(grid), property(NULL)
{
  wxString paramName = wxString::FromAscii(param->GetKey().c_str());
  wxString paramValue = wxString::FromAscii(param->GetAsString().c_str());
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Property::~Property()
{
  if (this->property && this->grid)
    this->grid->DeleteProperty( this->property );

  this->param = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// On changed 
void Property::Changed()
{
  std::cout << "Changed\n";

  std::string value = std::string(this->property->GetValueAsString().ToAscii());
  this->param->SetFromString( value, true );
}

////////////////////////////////////////////////////////////////////////////////
// FLOAT constructor
FloatProperty::FloatProperty(common::Param *p, wxPropertyGrid *grid)
  : Property(p, grid)
{
  this->property = this->grid->Append(
      new wxFloatProperty( wxString::FromAscii(param->GetKey().c_str()), 
                           wxPG_LABEL ));


  this->property->SetValueFromString( 
      wxString::FromAscii(param->GetAsString().c_str()) );
  this->property->SetClientData(this);
}

////////////////////////////////////////////////////////////////////////////////
// FLOAT destructor
FloatProperty::~FloatProperty()
{
}

////////////////////////////////////////////////////////////////////////////////
// DOUBLE constructor
DoubleProperty::DoubleProperty(common::Param *p, wxPropertyGrid *grid)
  : Property(p, grid)
{
  this->property = this->grid->Append(
      new wxFloatProperty( wxString::FromAscii(param->GetKey().c_str()), 
                            wxPG_LABEL ));

  this->property->SetValueFromString( 
      wxString::FromAscii(param->GetAsString().c_str()) );
  this->property->SetClientData(this);
}

////////////////////////////////////////////////////////////////////////////////
// DOUBLE destructor
DoubleProperty::~DoubleProperty()
{
}

////////////////////////////////////////////////////////////////////////////////
// INT constructor
IntProperty::IntProperty(common::Param *p, wxPropertyGrid *grid)
  : Property(p, grid)
{
  this->property = this->grid->Append(
      new wxIntProperty( wxString::FromAscii(param->GetKey().c_str()), 
                           wxPG_LABEL ));

  this->property->SetValueFromString( 
      wxString::FromAscii(param->GetAsString().c_str()) );
  this->property->SetClientData(this);
}

////////////////////////////////////////////////////////////////////////////////
// INT destructor
IntProperty::~IntProperty()
{
}

////////////////////////////////////////////////////////////////////////////////
// UINT constructor
UIntProperty::UIntProperty(common::Param *p, wxPropertyGrid *grid)
  : Property(p, grid)
{
  this->property = this->grid->Append(
      new wxUIntProperty( wxString::FromAscii(param->GetKey().c_str()), 
                           wxPG_LABEL ));

  this->property->SetValueFromString( 
      wxString::FromAscii(param->GetAsString().c_str()) );
  this->property->SetClientData(this);
}

////////////////////////////////////////////////////////////////////////////////
// UINT destructor
UIntProperty::~UIntProperty()
{
}

////////////////////////////////////////////////////////////////////////////////
// BOOL constructor
BoolProperty::BoolProperty(common::Param *p, wxPropertyGrid *grid)
  : Property(p, grid)
{
  this->property = this->grid->Append(
      new wxBoolProperty( wxString::FromAscii(param->GetKey().c_str()), 
                           wxPG_LABEL ));

  if (this->param->GetAsString() == "1")
    this->property->SetValueFromString( wxT("True") );
  else 
    this->property->SetValueFromString( wxT("False") );
  this->property->SetClientData(this);
}

////////////////////////////////////////////////////////////////////////////////
// BOOL destructor
BoolProperty::~BoolProperty()
{
}

////////////////////////////////////////////////////////////////////////////////
// STRING constructor
StringProperty::StringProperty(common::Param *p, wxPropertyGrid *grid)
  : Property(p, grid)
{
  this->property = this->grid->Append(
      new wxStringProperty( wxString::FromAscii(param->GetKey().c_str()), 
                           wxPG_LABEL ));

  this->property->SetValueFromString( 
      wxString::FromAscii(param->GetAsString().c_str()) );
  this->property->SetClientData(this);
}

////////////////////////////////////////////////////////////////////////////////
// STRING destructor
StringProperty::~StringProperty()
{
}

////////////////////////////////////////////////////////////////////////////////
// VECTOR3 constructor
Vector3Property::Vector3Property(common::Param *p, wxPropertyGrid *grid)
  : Property(p,grid)
{
  this->property = this->grid->Append(
      new wxStringProperty( wxString::FromAscii(param->GetKey().c_str()), 
                           wxPG_LABEL ));

  common::ParamT<common::Vector3> *pv = (common::ParamT<common::Vector3>*)this->param;

  this->property->SetValueFromString( 
      wxString::FromAscii(param->GetAsString().c_str()) );
  this->property->SetClientData(this);

  this->x = this->grid->AppendIn( this->property, new wxFloatProperty( wxT("X"),wxT("X"), (**pv).x) ); 
  this->y = this->grid->AppendIn( this->property, new wxFloatProperty( wxT("Y"), wxT("Y"), (**pv).y ) ); 
  this->z = this->grid->AppendIn( this->property, new wxFloatProperty( wxT("Z"), wxT("Z"), (**pv).z) ); 

  this->grid->Collapse( this->property );
}

////////////////////////////////////////////////////////////////////////////////
// VECTOR3 destructor
Vector3Property::~Vector3Property()
{
}

////////////////////////////////////////////////////////////////////////////////
// QUATERN constructor
QuaternProperty::QuaternProperty(common::Param *p, wxPropertyGrid *grid)
  : Property(p,grid)
{
  this->property = this->grid->Append(
      new wxStringProperty( wxString::FromAscii(param->GetKey().c_str()), 
                           wxPG_LABEL ));
  this->property->SetClientData(this);

  this->property->SetValueFromString( 
      wxString::FromAscii(param->GetAsString().c_str()) );

  common::ParamT<common::Quatern> *pv = (common::ParamT<common::Quatern>*)this->param;
  common::Vector3 rpy = (**pv).GetAsEuler();

  this->roll = this->grid->AppendIn( this->property, new wxFloatProperty( wxT("Roll"), wxT("Roll"), rpy.x) ); 
  this->pitch = this->grid->AppendIn( this->property, new wxFloatProperty( wxT("Pitch"), wxT("Pitch"), rpy.y ) ); 
  this->yaw = this->grid->AppendIn( this->property, new wxFloatProperty( wxT("Yaw"), wxT("Yaw"), rpy.z) ); 

  this->grid->Collapse( this->property );
}

////////////////////////////////////////////////////////////////////////////////
// QUATERN destructor
QuaternProperty::~QuaternProperty()
{
}

////////////////////////////////////////////////////////////////////////////////
// TIME constructor
TimeProperty::TimeProperty(common::Param *p, wxPropertyGrid *grid)
  : Property(p,grid)
{
  this->property = this->grid->Append(
      new wxStringProperty( wxString::FromAscii(param->GetKey().c_str()), 
                           wxPG_LABEL ));
  this->property->SetClientData(this);

  this->property->SetValueFromString( 
      wxString::FromAscii(param->GetAsString().c_str()) );

  this->sec = this->grid->AppendIn( this->property, new wxIntProperty( wxT("Seconds"), wxT("Sec"), 0.0) ); 
  this->msec = this->grid->AppendIn( this->property, new wxIntProperty( wxT("Miliseconds"), wxT("Millisec"), 0.0) ); 

  this->grid->Collapse( this->property );
}

////////////////////////////////////////////////////////////////////////////////
// TIME destructor
TimeProperty::~TimeProperty()
{
}

////////////////////////////////////////////////////////////////////////////////
// COLOR constructor
ColorProperty::ColorProperty(common::Param *p, wxPropertyGrid *grid)
  : Property(p,grid)
{
  this->property = this->grid->Append(
      new wxStringProperty( wxString::FromAscii(param->GetKey().c_str()), 
                           wxPG_LABEL ));
  this->property->SetClientData(this);

  this->property->SetValueFromString( 
      wxString::FromAscii(param->GetAsString().c_str()) );
}

////////////////////////////////////////////////////////////////////////////////
// COLOR destructor
ColorProperty::~ColorProperty()
{
}
