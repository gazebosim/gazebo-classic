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

void Element::AddAttribute(const std::string &_key, const std::string &_type,
                              const std::string &_defaultValue, bool _required)
{
  if (_type == "double")
  {
    boost::shared_ptr<ParamT<double> > param( 
        new ParamT<double>(_key, _defaultValue,_required) );
    this->attributes.push_back(boost::shared_static_cast<Param>(param));
  }
  else if (_type == "int")
  {
    boost::shared_ptr<ParamT<int> > param( 
        new ParamT<int>(_key,_defaultValue,_required) );
    this->attributes.push_back(param);
  }
  else if (_type == "unsigned int")
  {
    boost::shared_ptr<ParamT<unsigned int> > param( 
        new ParamT<unsigned int>(_key,_defaultValue,_required) );
    this->attributes.push_back(param);
  }
  else if (_type == "float")
  {
    boost::shared_ptr<ParamT<float> > param( 
        new ParamT<float>(_key,_defaultValue,_required) );
    this->attributes.push_back(param);
  }
  else if (_type == "bool")
  {
    boost::shared_ptr<ParamT<bool> > param( 
        new ParamT<bool>(_key,_defaultValue,_required) );
    this->attributes.push_back(param);
  }
  else if (_type == "string")
  {
    boost::shared_ptr<ParamT<std::string> > param( 
        new ParamT<std::string>(_key,_defaultValue,_required) );
    this->attributes.push_back(param);
  }
  else if (_type == "color")
  {
    boost::shared_ptr<ParamT<gazebo::common::Color> > param( 
        new ParamT<gazebo::common::Color>(_key,_defaultValue,_required) );
    this->attributes.push_back(param);
  }
  else if (_type == "vector3")
  {
    boost::shared_ptr<ParamT<gazebo::math::Vector3> > param( 
        new ParamT<gazebo::math::Vector3>(_key,_defaultValue,_required) );
    this->attributes.push_back(param);
  }
  else if (_type == "pose")
  {
    boost::shared_ptr<ParamT<gazebo::math::Pose> > param( 
        new ParamT<gazebo::math::Pose>(_key,_defaultValue,_required) );
    this->attributes.push_back(param);
  }
  else
  {
    gzerr << "Unknown attribute _type[" << _type << "]\n";
  }
}


ElementPtr Element::Clone() const
{
  ElementPtr clone(new Element);
  clone->name = this->name;
  clone->required = this->required;

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
  gzerr << "Unable to find attribute [" << _key << "]\n";
  return ParamPtr();
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
      this->elements.push_back( (*iter)->Clone() );
      return this->elements.back();
    }
  }
  gzerr << "Missing element description for [" << _name << "]\n";
  return ElementPtr();
}

bool Element::GetValueBool(const std::string &_key)
{
  bool result;
  ParamPtr param = this->GetAttribute(_key);
  if (param)
    param->Get(result);
  else
    gzerr << "Unable to find value for key[" << _key << "]\n";
  return result;
}

int Element::GetValueInt(const std::string &_key)
{
  int result;
  ParamPtr param = this->GetAttribute(_key);
  if (param)
    param->Get(result);
  else
    gzerr << "Unable to find value for key[" << _key << "]\n";
  return result;
}
float Element::GetValueFloat(const std::string &_key)
{
  float result;
  ParamPtr param = this->GetAttribute(_key);
  if (param)
    param->Get(result);
  else
    gzerr << "Unable to find value for key[" << _key << "]\n";
  return result;
}
double Element::GetValueDouble(const std::string &_key)
{
  double result;
  ParamPtr param = this->GetAttribute(_key);
  if (param)
    param->Get(result);
  else
    gzerr << "Unable to find value for key[" << _key << "]\n";
  return result;
}
unsigned int Element::GetValueUInt(const std::string &_key)
{
  unsigned int result;
  ParamPtr param = this->GetAttribute(_key);
  if (param)
    param->Get(result);
  else
    gzerr << "Unable to find value for key[" << _key << "]\n";
  return result;
}
char Element::GetValueChar(const std::string &_key)
{
  char result;
  ParamPtr param = this->GetAttribute(_key);
  if (param)
    param->Get(result);
  else
    gzerr << "Unable to find value for key[" << _key << "]\n";
  return result;
}
std::string Element::GetValueString(const std::string &_key)
{
  std::string result;
  ParamPtr param = this->GetAttribute(_key);
  if (param)
    param->Get(result);
  else
    gzerr << "Unable to find value for key[" << _key << "]\n";
  return result;
}
gazebo::math::Vector3 Element::GetValueVector3(const std::string &_key)
{
  gazebo::math::Vector3 result;
  ParamPtr param = this->GetAttribute(_key);
  if (param)
    param->Get(result);
  else
    gzerr << "Unable to find value for key[" << _key << "]\n";
  return result;
}
gazebo::math::Quaternion Element::GetValueQuaternion(const std::string &_key)
{
  gazebo::math::Quaternion result;
  ParamPtr param = this->GetAttribute(_key);
  if (param)
    param->Get(result);
  else
    gzerr << "Unable to find value for key[" << _key << "]\n";
  return result;
}

gazebo::math::Pose Element::GetValuePose(const std::string &_key)
{
  gazebo::math::Pose result;
  ParamPtr param = this->GetAttribute(_key);
  if (param)
    param->Get(result);
  else
    gzerr << "Unable to find value for key[" << _key << "]\n";
  return result;
}

gazebo::common::Color Element::GetValueColor(const std::string &_key)
{
  gazebo::common::Color result;
  ParamPtr param = this->GetAttribute(_key);
  if (param)
    param->Get(result);
  else
    gzerr << "Unable to find value for key[" << _key << "]\n";
  return result;
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
