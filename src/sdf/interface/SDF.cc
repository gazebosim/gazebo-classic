#include "common/Color.hh"
#include "math/Pose.hh"
#include "math/Vector3.hh"

#include "sdf/interface/SDF.hh"

using namespace sdf;

void SDFElement::SetName(const std::string &_name)
{
  this->name = _name;
}

const std::string &SDFElement::GetName() const
{
  return this->name;
}

void SDFElement::SetRequired(const std::string &_req)
{
  this->required = _req;
}

const std::string &SDFElement::GetRequired() const
{
  return this->required;
}

void SDFElement::AddAttribute(const std::string &_key, const std::string &_type,
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


boost::shared_ptr<SDFElement> SDFElement::Clone() const
{
  boost::shared_ptr<SDFElement> clone(new SDFElement);
  clone->name = this->name;
  clone->required = this->required;

  std::vector< boost::shared_ptr<Param> >::const_iterator aiter;
  for (aiter = this->attributes.begin(); 
      aiter != this->attributes.end(); aiter++)
  {
    clone->attributes.push_back((*aiter)->Clone());
  }

  std::vector< boost::shared_ptr<SDFElement> >::const_iterator eiter;
  for (eiter = this->elementDescriptions.begin(); 
      eiter != this->elementDescriptions.end(); eiter++)
  {
    clone->elementDescriptions.push_back((*eiter)->Clone());
  }

  return clone;
}

void SDFElement::PrintDescription(std::string _prefix)
{
  std::cout << _prefix << "<element name='" << this->name << "' required='" << this->required << "'>\n";

  std::vector< boost::shared_ptr<Param> >::iterator aiter;
  for (aiter = this->attributes.begin(); 
      aiter != this->attributes.end(); aiter++)
  {
    std::cout << _prefix << "  <attribute name='" << (*aiter)->GetKey() << "' type='" << (*aiter)->GetTypeName() << "' default='" << (*aiter)->GetDefaultAsString() << "' required='" << (*aiter)->GetRequired() << "'/>\n";
  }

  std::vector< boost::shared_ptr<SDFElement> >::iterator eiter;
  for (eiter = this->elementDescriptions.begin(); 
      eiter != this->elementDescriptions.end(); eiter++)
  {
    (*eiter)->PrintDescription(_prefix + "  ");
  }

  std::cout << _prefix << "</element>\n";
}

void SDFElement::PrintValues(std::string _prefix)
{
  std::cout << _prefix << "<" << this->name << " ";

  std::vector< boost::shared_ptr<Param> >::iterator aiter;
  for (aiter = this->attributes.begin(); 
      aiter != this->attributes.end(); aiter++)
  {
    std::cout << (*aiter)->GetKey() << "='" 
      << (*aiter)->GetAsString() << "' ";
  }

  if(this->elements.size() > 0)
  {
    std::cout << ">\n";
    std::vector< boost::shared_ptr<SDFElement> >::iterator eiter;
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

boost::shared_ptr<Param> SDFElement::GetAttribute(const std::string &_key)
{
  std::vector< boost::shared_ptr<Param> >::const_iterator iter;
  for (iter = this->attributes.begin(); 
      iter != this->attributes.end(); iter++)
  {
    if ( (*iter)->GetKey() == _key)
      return (*iter);
  }
  gzerr << "Unable to find attribute [" << _key << "]\n";
  return boost::shared_ptr<Param>();
}

boost::shared_ptr<SDFElement> SDFElement::AddElement(const std::string &_name)
{
  std::vector< boost::shared_ptr<SDFElement> >::const_iterator iter;
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
  return boost::shared_ptr<SDFElement>();
}

SDF::SDF()
  : root(new SDFElement)
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
