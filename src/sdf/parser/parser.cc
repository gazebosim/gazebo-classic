
#include "sdf/interface/sdf.h"
#include "sdf/parser/parser.hh"

#include "common/Console.hh"
#include "common/Color.hh"
#include "math/Vector3.hh"
#include "math/Pose.hh"

namespace sdf
{

////////////////////////////////////////////////////////////////////////////////
/// Load the sdf format from a file
bool initFile(const std::string &_filename, boost::shared_ptr<SDF> _sdf)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);
  return initDoc(&xmlDoc, _sdf);
}

////////////////////////////////////////////////////////////////////////////////
/// Load the sdf format from a string
bool initString(const std::string &_xmlString, boost::shared_ptr<SDF> &_sdf)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());

  return initDoc(&xmlDoc, _sdf);
}

////////////////////////////////////////////////////////////////////////////////
/// Load the sdf format from TiXMLDocument
bool initDoc(TiXmlDocument *_xmlDoc, boost::shared_ptr<SDF> &_sdf)
{
  if (!_xmlDoc)
  {
    gzerr << "Could not parse the xml\n";
    return false;
  }

  TiXmlElement *xml = _xmlDoc->FirstChildElement("element");
  if (!xml)
  {
    gzerr << "Could not find the 'element' element in the xml file\n";
    return false;
  }
  boost::shared_ptr<SDFElement> element(new SDFElement);
  _sdf->elementDescriptions.push_back(element);

  return initXml(xml, element);
}

////////////////////////////////////////////////////////////////////////////////
// Load an SDF element from XML
bool initXml(TiXmlElement *_xml, boost::shared_ptr<SDFElement> &_sdf)
{
  const char *nameString = _xml->Attribute("name");
  if (!nameString)
  {
    gzerr << "Element is missing the name attribute\n";
    return false;
  }
  _sdf->name = std::string(nameString);

  const char *requiredString = _xml->Attribute("required");
  if (!requiredString)
  {
    gzerr << "Element is missing the required attributed\n";
    return false;
  }
  _sdf->required = requiredString;
  
  // Get all attributes
  for (TiXmlElement *child = _xml->FirstChildElement("attribute"); 
      child; child = child->NextSiblingElement("attribute"))
  {
    const char *name = child->Attribute("name");
    const char *type = child->Attribute("type");
    const char *defaultValue = child->Attribute("default");
    const char *requiredString = child->Attribute("required");
    if (!name)
    {
      gzerr << "Attribute is missing a name\n";
      return false;
    }
    if (!type)
    {
      gzerr << "Attribute is missing a type\n";
      return false;
    }
    if (!defaultValue)
    {
      gzerr << "Attribute[" << name << "] is missing a default\n";
      return false;
    }
    if (!requiredString)
    {
      gzerr << "Attribute is missing a required string\n";
      return false;
    }
    bool required = std::string(requiredString) == "1" ? true : false;

    if (std::string(type) == "double")
    {
      boost::shared_ptr<ParamT<double> > param( 
          new ParamT<double>(name,defaultValue,required) );
      _sdf->attributes.push_back(boost::shared_static_cast<Param>(param));
    }
    else if (std::string(type) == "int")
    {
      boost::shared_ptr<ParamT<int> > param( 
          new ParamT<int>(name,defaultValue,required) );
      _sdf->attributes.push_back(param);
    }
    else if (std::string(type) == "unsigned int")
    {
      boost::shared_ptr<ParamT<unsigned int> > param( 
          new ParamT<unsigned int>(name,defaultValue,required) );
      _sdf->attributes.push_back(param);
    }
    else if (std::string(type) == "float")
    {
      boost::shared_ptr<ParamT<float> > param( 
          new ParamT<float>(name,defaultValue,required) );
      _sdf->attributes.push_back(param);
    }
    else if (std::string(type) == "bool")
    {
      boost::shared_ptr<ParamT<bool> > param( 
          new ParamT<bool>(name,defaultValue,required) );
      _sdf->attributes.push_back(param);
    }
    else if (std::string(type) == "string")
    {
      boost::shared_ptr<ParamT<std::string> > param( 
          new ParamT<std::string>(name,defaultValue,required) );
      _sdf->attributes.push_back(param);
    }
    else if (std::string(type) == "color")
    {
      boost::shared_ptr<ParamT<gazebo::common::Color> > param( 
          new ParamT<gazebo::common::Color>(name,defaultValue,required) );
      _sdf->attributes.push_back(param);
    }
    else if (std::string(type) == "vector3")
    {
      boost::shared_ptr<ParamT<gazebo::math::Vector3> > param( 
          new ParamT<gazebo::math::Vector3>(name,defaultValue,required) );
      _sdf->attributes.push_back(param);
    }
    else if (std::string(type) == "pose")
    {
      boost::shared_ptr<ParamT<gazebo::math::Pose> > param( 
          new ParamT<gazebo::math::Pose>(name,defaultValue,required) );
      _sdf->attributes.push_back(param);
    }
    else
    {
      gzerr << "Unknown attribute type[" << type << "]\n";
      return false;
    }
  }
  
  // Get all child elements
  for (TiXmlElement *child = _xml->FirstChildElement("element"); 
       child; child = child->NextSiblingElement("element"))
  {
    boost::shared_ptr<SDFElement> element(new SDFElement);
    initXml(child, element);
    _sdf->elementDescriptions.push_back(element);
  }

  return true;
}


bool readFile(const std::string &_filename, boost::shared_ptr<SDF> &_sdf)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);
  return readDoc(&xmlDoc, _sdf);
}

bool readString(const std::string &_xmlString, boost::shared_ptr<SDF> &_sdf)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());

  return readDoc(&xmlDoc, _sdf);
}

bool readDoc(TiXmlDocument *_xmlDoc, boost::shared_ptr<SDF> &_sdf)
{
  if (!_xmlDoc)
  {
    gzerr << "Could not parse the xml\n";
    return false;
  }

  std::vector< boost::shared_ptr<SDFElement> >::iterator iter;
  for (iter = _sdf->elementDescriptions.begin(); 
       iter != _sdf->elementDescriptions.end(); iter++)
  {
    for (TiXmlElement* elemXml = _xmlDoc->FirstChildElement((*iter)->name);
         elemXml; elemXml = elemXml->NextSiblingElement((*iter)->name))
    {
      boost::shared_ptr<SDFElement> element = (*iter)->Clone();
      if (!readXml( elemXml, element ))
      {
        gzerr << "Unable to parse sdf element[" << element->name << "]\n";
        return false;
      }
      _sdf->elements.push_back(element);
      if ((*iter)->required == "0" || (*iter)->required == "1")
        break;
    }
  }

  return true;
}

bool readXml(TiXmlElement *_xml, boost::shared_ptr<SDFElement> &_sdf)
{
  if (!_xml)
  {
    if (_sdf->required == "1" || _sdf->required=="+")
    {
      gzerr << "SDF Element[" << _sdf->name << "] is missing\n";
      return false;
    }
    else
      return true;
  }

  // Read all the attributes
  std::vector< boost::shared_ptr<Param> >::iterator iter;
  for (iter = _sdf->attributes.begin(); 
       iter != _sdf->attributes.end(); iter++)
  {
    const char *str = _xml->Attribute( (*iter)->GetKey().c_str() );
    if ( !(*iter)->Set( str ))
    {
      gzerr << "Unable to read attribute[" << (*iter)->GetKey() << "]\n";
      return false;
    }
  }

  // Read all the elements
  std::vector< boost::shared_ptr<sdf::SDFElement> >::iterator eiter;
  for (eiter = _sdf->elementDescriptions.begin(); 
       eiter != _sdf->elementDescriptions.end(); eiter++)
  {
    for (TiXmlElement* elemXml = _xml->FirstChildElement((*eiter)->name);
         elemXml; elemXml = elemXml->NextSiblingElement((*eiter)->name))
    {
      boost::shared_ptr<sdf::SDFElement> element = (*eiter)->Clone();
      readXml( elemXml, element );
      _sdf->elements.push_back(element);
      if ((*eiter)->required == "0" || (*eiter)->required == "1")
        break;
    }
  }
  return true;
}

}
