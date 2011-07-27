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
#include "common/Color.hh"
#include "math/Pose.hh"
#include "math/Vector3.hh"

#include "sdf/interface/SDF.hh"

using namespace sdf;

ElementPtr Element::GetParent() const
{
  return this->parent;
}

void Element::SetParent(const ElementPtr &_parent)
{
  this->parent = _parent;
}

/// Get the name of the parent link. 
std::string Element::GetLinkName() const
{
  std::string result;

  // get parent body name by looking through sdf
  sdf::ElementPtr sdfParent = this->GetParent();
  while (sdfParent && sdfParent->GetName() != "link")
    sdfParent = sdfParent->GetParent();

  if (sdfParent)
   result = sdfParent->GetValueString("name");

  return result;
}

/// Get the name of the parent model. 
std::string Element::GetModelName() const
{
  std::string result;

  // get parent body name by looking through sdf
  sdf::ElementPtr sdfParent = this->GetParent();
  while (sdfParent && sdfParent->GetName() != "model")
    sdfParent = sdfParent->GetParent();

  if (sdfParent)
   result = sdfParent->GetValueString("name");

  return result;
}

/// Get the name of the parent world. 
std::string Element::GetWorldName() const
{
  std::string result;

  // get parent body name by looking through sdf
  sdf::ElementPtr sdfParent = this->GetParent();
  while (sdfParent && sdfParent->GetName() != "world")
    sdfParent = sdfParent->GetParent();

  if (sdfParent)
   result = sdfParent->GetValueString("name");

  return result;
}


void Element::SetName(const std::string &_name)
{
  this->name = _name;
}

const std::string &Element::GetName() const
{
  return this->name;
}

void Element::SetRequired(const std::string &_req)
{
  this->required = _req;
}

const std::string &Element::GetRequired() const
{
  return this->required;
}

void Element::AddValue(const std::string &_type, 
                       const std::string &_defaultValue, bool _required)
{
  this->value = this->CreateParam(this->name, _type, _defaultValue, _required);
}

boost::shared_ptr<Param> Element::CreateParam(const std::string &_key, 
    const std::string &_type, const std::string &_defaultValue, bool _required)
{
  if (_type == "double")
  {
    boost::shared_ptr<ParamT<double> > param( 
        new ParamT<double>(_key, _defaultValue,_required) );
    return param;
  }
  else if (_type == "int")
  {
    boost::shared_ptr<ParamT<int> > param( 
        new ParamT<int>(_key,_defaultValue,_required) );
    return param;
  }
  else if (_type == "unsigned int")
  {
    boost::shared_ptr<ParamT<unsigned int> > param( 
        new ParamT<unsigned int>(_key,_defaultValue,_required) );
    return param;
  }
  else if (_type == "float")
  {
    boost::shared_ptr<ParamT<float> > param( 
        new ParamT<float>(_key,_defaultValue,_required) );
    return param;
  }
  else if (_type == "bool")
  {
    boost::shared_ptr<ParamT<bool> > param( 
        new ParamT<bool>(_key,_defaultValue,_required) );
    return param;
  }
  else if (_type == "string")
  {
    boost::shared_ptr<ParamT<std::string> > param( 
        new ParamT<std::string>(_key,_defaultValue,_required) );
    return param;
  }
  else if (_type == "color")
  {
    boost::shared_ptr<ParamT<gazebo::common::Color> > param( 
        new ParamT<gazebo::common::Color>(_key,_defaultValue,_required) );
    return param;
  }
  else if (_type == "vector3")
  {
    boost::shared_ptr<ParamT<gazebo::math::Vector3> > param( 
        new ParamT<gazebo::math::Vector3>(_key,_defaultValue,_required) );
    return param;
  }
  else if (_type == "pose")
  {
    boost::shared_ptr<ParamT<gazebo::math::Pose> > param( 
        new ParamT<gazebo::math::Pose>(_key,_defaultValue,_required) );
    return param;
  }
  else
  {
    gzerr << "Unknown attribute _type[" << _type << "]\n";
    return boost::shared_ptr<Param>();
  }
}

void Element::AddAttribute(const std::string &_key, const std::string &_type,
                              const std::string &_defaultValue, bool _required)
{
  this->attributes.push_back(
      this->CreateParam(_key, _type, _defaultValue, _required));
}

ElementPtr Element::Clone() const
{
  ElementPtr clone(new Element);
  clone->name = this->name;
  clone->required = this->required;
  clone->parent = this->parent;

  Param_V::const_iterator aiter;
  for (aiter = this->attributes.begin(); 
      aiter != this->attributes.end(); aiter++)
  {
    clone->attributes.push_back((*aiter)->Clone());
  }

  ElementPtr_V::const_iterator eiter;
  for (eiter = this->elementDescriptions.begin(); 
       eiter != this->elementDescriptions.end(); eiter++)
  {
    clone->elementDescriptions.push_back((*eiter)->Clone());
  }

  if (this->value)
    clone->value = this->value->Clone();
  return clone;
}

void Element::PrintDescription(std::string _prefix)
{
  std::cout << _prefix << "<element name='" << this->name << "' required='" << this->required << "'>\n";

  Param_V::iterator aiter;
  for (aiter = this->attributes.begin(); 
      aiter != this->attributes.end(); aiter++)
  {
    std::cout << _prefix << "  <attribute name='" << (*aiter)->GetKey() << "' type='" << (*aiter)->GetTypeName() << "' default='" << (*aiter)->GetDefaultAsString() << "' required='" << (*aiter)->GetRequired() << "'/>\n";
  }

  ElementPtr_V::iterator eiter;
  for (eiter = this->elementDescriptions.begin(); 
      eiter != this->elementDescriptions.end(); eiter++)
  {
    (*eiter)->PrintDescription(_prefix + "  ");
  }

  std::cout << _prefix << "</element>\n";
}

void Element::PrintValues(std::string _prefix)
{
  std::cout << _prefix << "<" << this->name << " ";

  Param_V::iterator aiter;
  for (aiter = this->attributes.begin(); 
      aiter != this->attributes.end(); aiter++)
  {
    std::cout << (*aiter)->GetKey() << "='" 
      << (*aiter)->GetAsString() << "' ";
  }

  if(this->elements.size() > 0)
  {
    std::cout << ">\n";
    ElementPtr_V::iterator eiter;
    for (eiter = this->elements.begin(); 
        eiter != this->elements.end(); eiter++)
    {
      (*eiter)->PrintValues(_prefix + "  ");
    }
    std::cout << _prefix << "</" << this->name << ">\n";
  }
  else
    std::cout << "/>\n";
}



ParamPtr Element::GetAttribute(const std::string &_key)
{
  Param_V::const_iterator iter;
  for (iter = this->attributes.begin(); 
      iter != this->attributes.end(); iter++)
  {
    if ( (*iter)->GetKey() == _key)
      return (*iter);
  }
  gzerr << "Element[" << this->GetName() << "] does not have attribute [" << _key << "]\n";
  return ParamPtr();
}

/// Get the param of the elements value
ParamPtr Element::GetValue()
{
  return this->value;
}

bool Element::HasElement(const std::string &_name) const
{
  ElementPtr_V::const_iterator iter;
  for (iter = this->elements.begin(); iter != this->elements.end(); iter++)
  {
    if ( (*iter)->GetName() == _name )
      return true;
  }

  return false;
}

ElementPtr Element::GetElement(const std::string &_name) const
{
  ElementPtr_V::const_iterator iter;
  for (iter = this->elements.begin(); iter != this->elements.end(); iter++)
  {
    if ( (*iter)->GetName() == _name )
      return (*iter);
  }

  gzerr << "Unable to find element with name[" << _name << "]\n";
  return ElementPtr();
}

ElementPtr Element::GetFirstElement() const
{
  return this->elements.front();
}

ElementPtr Element::GetNextElement(const std::string &_name, 
                          const ElementPtr &_elem) const
{
  ElementPtr_V::const_iterator iter;
  iter = std::find(this->elements.begin(), this->elements.end(), _elem);

  for (iter++; iter != this->elements.end(); iter++)
  {
    if ( (*iter)->GetName() == _name )
      return (*iter);
  }

  return ElementPtr();
}

boost::shared_ptr<Element> Element::GetOrCreateElement(const std::string &_name)
{
  if (this->HasElement(_name))
    return this->GetElement(_name);
  else
    return this->AddElement(_name);
}

ElementPtr Element::AddElement(const std::string &_name)
{
  ElementPtr_V::const_iterator iter;
  for (iter = this->elementDescriptions.begin(); 
      iter != this->elementDescriptions.end(); iter++)
  {
    if ((*iter)->name == _name)
    {
      ElementPtr elem = (*iter)->Clone();
      elem->SetParent( shared_from_this() );
      this->elements.push_back( elem );
      return this->elements.back();
    }
  }
  gzerr << "Missing element description for [" << _name << "]\n";
  return ElementPtr();
}

bool Element::GetValueBool(const std::string &_key)
{
  bool result;

  if (_key.empty())
    this->value->Get(result);
  else
  {
    ParamPtr param = this->GetAttribute(_key);
    if (param)
      param->Get(result);
    else
      gzerr << "Unable to find value for key[" << _key << "]\n";
  }
  return result;
}

int Element::GetValueInt(const std::string &_key)
{
  int result;
  if (_key.empty())
    this->value->Get(result);
  else
  {

    ParamPtr param = this->GetAttribute(_key);
    if (param)
      param->Get(result);
    else
      gzerr << "Unable to find value for key[" << _key << "]\n";
  }
  return result;
}
float Element::GetValueFloat(const std::string &_key)
{
  float result;
  if (_key.empty())
    this->value->Get(result);
  else
  {
    ParamPtr param = this->GetAttribute(_key);
    if (param)
      param->Get(result);
    else
      gzerr << "Unable to find value for key[" << _key << "]\n";
  }
  return result;
}
double Element::GetValueDouble(const std::string &_key)
{
  double result;
  if (_key.empty())
    this->value->Get(result);
  else
  {
    ParamPtr param = this->GetAttribute(_key);
    if (param)
      param->Get(result);
    else
      gzerr << "Unable to find value for key[" << _key << "]\n";
  }
  return result;
}
unsigned int Element::GetValueUInt(const std::string &_key)
{
  unsigned int result;
  if (_key.empty())
    this->value->Get(result);
  else
  {
    ParamPtr param = this->GetAttribute(_key);
    if (param)
      param->Get(result);
    else
      gzerr << "Unable to find value for key[" << _key << "]\n";
  }
  return result;
}
char Element::GetValueChar(const std::string &_key)
{
  char result;
  if (_key.empty())
    this->value->Get(result);
  else
  {
    ParamPtr param = this->GetAttribute(_key);
    if (param)
      param->Get(result);
    else
      gzerr << "Unable to find value for key[" << _key << "]\n";
  }
  return result;
}
std::string Element::GetValueString(const std::string &_key)
{
  std::string result;
  if (_key.empty())
    this->value->Get(result);
  else
  {
    ParamPtr param = this->GetAttribute(_key);
    if (param)
      param->Get(result);
    else
      gzerr << "Unable to find value for key[" << _key << "]\n";
  }
  return result;
}
gazebo::math::Vector3 Element::GetValueVector3(const std::string &_key)
{
  gazebo::math::Vector3 result;
  if (_key.empty())
    this->value->Get(result);
  else
  {
    ParamPtr param = this->GetAttribute(_key);
    if (param)
      param->Get(result);
    else
      gzerr << "Unable to find value for key[" << _key << "]\n";
  }
  return result;
}
gazebo::math::Quaternion Element::GetValueQuaternion(const std::string &_key)
{
  gazebo::math::Quaternion result;
  if (_key.empty())
    this->value->Get(result);
  else
  {
    ParamPtr param = this->GetAttribute(_key);
    if (param)
      param->Get(result);
    else
      gzerr << "Unable to find value for key[" << _key << "]\n";
  }
  return result;
}

gazebo::math::Pose Element::GetValuePose(const std::string &_key)
{
  gazebo::math::Pose result;
  if (_key.empty())
    this->value->Get(result);
  else
  {
    ParamPtr param = this->GetAttribute(_key);
    if (param)
      param->Get(result);
    else
      gzerr << "Unable to find value for key[" << _key << "]\n";
  }
  return result;
}

gazebo::common::Color Element::GetValueColor(const std::string &_key)
{
  gazebo::common::Color result;
  if (_key.empty())
    this->value->Get(result);
  else
  {
    ParamPtr param = this->GetAttribute(_key);
    if (param)
      param->Get(result);
    else
      gzerr << "Unable to find value for key[" << _key << "]\n";
  }
  return result;
}
void Element::ClearElements()
{
  this->elements.clear();
}

SDF::SDF()
  : root(new Element)
{
}

void SDF::PrintDescription()
{
  this->root->PrintDescription("");
}

void SDF::PrintValues()
{
  this->root->PrintValues("");
}
