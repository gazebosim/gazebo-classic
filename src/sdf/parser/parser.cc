
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

  return initXml(xml, _sdf->root);
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
  _sdf->SetName( std::string(nameString) );

  const char *requiredString = _xml->Attribute("required");
  if (!requiredString)
  {
    gzerr << "Element is missing the required attributed\n";
    return false;
  }
  _sdf->SetRequired( requiredString );
  
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

    _sdf->AddAttribute(name, type, defaultValue, required);
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

  TiXmlElement* elemXml = _xmlDoc->FirstChildElement(_sdf->root->GetName());
   
  if (!readXml( elemXml, _sdf->root))
  {
    gzerr << "Unable to parse sdf element[" << _sdf->root->GetName() << "]\n";
    return false;
  }

  return true;
}

bool readXml(TiXmlElement *_xml, boost::shared_ptr<SDFElement> &_sdf)
{
  if (!_xml)
  {
    if (_sdf->GetRequired() == "1" || _sdf->GetRequired() =="+")
    {
      gzerr << "SDF Element[" << _sdf->GetName() << "] is missing\n";
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
    if ( str && !(*iter)->Set( str ))
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
    for (TiXmlElement* elemXml = _xml->FirstChildElement((*eiter)->GetName());
         elemXml; elemXml = elemXml->NextSiblingElement((*eiter)->GetName()))
    {
      boost::shared_ptr<sdf::SDFElement> element = (*eiter)->Clone();
      readXml( elemXml, element );
      _sdf->elements.push_back(element);
      if ((*eiter)->GetRequired() == "0" || (*eiter)->GetRequired() == "1")
        break;
    }
  }

  return true;
}

}
