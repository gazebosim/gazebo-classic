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
#include "sdf/sdf.h"
#include "sdf/sdf_parser.h"

#include "common/SystemPaths.hh"
#include "common/Console.hh"
#include "common/Color.hh"
#include "math/Vector3.hh"
#include "math/Pose.hh"

namespace sdf
{

////////////////////////////////////////////////////////////////////////////////
/// Init based on the installed sdf_format.xml file
bool init( SDFPtr _sdf )
{
  const std::list<std::string> paths = gazebo::common::SystemPaths::Instance()->GetGazeboPaths();
  std::list<std::string>::const_iterator iter;
  for ( iter = paths.begin(); iter != paths.end(); iter++)
  {
    std::string filename = (*iter) + "/sdf_format.xml";
    FILE *ftest = fopen(filename.c_str(), "r");
    if (ftest && initFile(filename, _sdf))
        return true;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////
bool initFile(const std::string &_filename, SDFPtr _sdf)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);
  return initDoc(&xmlDoc, _sdf);
}

////////////////////////////////////////////////////////////////////////////////
bool initString(const std::string &_xmlString, SDFPtr &_sdf)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());

  return initDoc(&xmlDoc, _sdf);
}

////////////////////////////////////////////////////////////////////////////////
bool initDoc(TiXmlDocument *_xmlDoc, SDFPtr &_sdf)
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
bool initXml(TiXmlElement *_xml, ElementPtr &_sdf)
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
 
  const char *elemTypeString = _xml->Attribute("type");
  if (elemTypeString)
  {
    bool required = std::string(requiredString) == "1" ? true : false;
    const char *elemDefaultValue = _xml->Attribute("default");
    _sdf->AddValue(elemTypeString, elemDefaultValue, required);
  }
  
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
    ElementPtr element(new Element);
    initXml(child, element);
    _sdf->elementDescriptions.push_back(element);
  }

  return true;
}


////////////////////////////////////////////////////////////////////////////////
bool readFile(const std::string &_filename, SDFPtr &_sdf)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);
  return readDoc(&xmlDoc, _sdf);
}

////////////////////////////////////////////////////////////////////////////////
bool readString(const std::string &_xmlString, SDFPtr &_sdf)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());
  return readDoc(&xmlDoc, _sdf);
}

////////////////////////////////////////////////////////////////////////////////
bool readDoc(TiXmlDocument *_xmlDoc, SDFPtr &_sdf)
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

////////////////////////////////////////////////////////////////////////////////
bool readXml(TiXmlElement *_xml, ElementPtr &_sdf)
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

  if (_xml->GetText() != NULL && _sdf->value)
  {
    _sdf->value->SetFromString( _xml->GetText() );
  }

  // Read all the attributes
  Param_V::iterator iter;
  for (iter = _sdf->attributes.begin(); 
       iter != _sdf->attributes.end(); iter++)
  {
    const char *str = _xml->Attribute( (*iter)->GetKey().c_str() );
    if ( str && !(*iter)->SetFromString( str ))
    {
      gzerr << "Unable to read attribute[" << (*iter)->GetKey() << "]\n";
      return false;
    }
  }

  // Read all the elements
  ElementPtr_V::iterator eiter;
  for (eiter = _sdf->elementDescriptions.begin(); 
       eiter != _sdf->elementDescriptions.end(); eiter++)
  {
    for (TiXmlElement* elemXml = _xml->FirstChildElement((*eiter)->GetName());
         elemXml; elemXml = elemXml->NextSiblingElement((*eiter)->GetName()))
    {
      ElementPtr element = (*eiter)->Clone();
      readXml( elemXml, element );
      _sdf->elements.push_back(element);
      if ((*eiter)->GetRequired() == "0" || (*eiter)->GetRequired() == "1")
        break;
    }
  }

  return true;
}

}
