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
#include <stdlib.h>

#include "sdf/sdf.h"
#include "sdf/sdf_parser.h"

#include "parser_deprecated.hh"

#include "common/Console.hh"
#include "common/Color.hh"
#include "math/Vector3.hh"
#include "math/Pose.hh"
#include "common/SystemPaths.hh"
#include "common/Exception.hh"

namespace sdf
{

////////////////////////////////////////////////////////////////////////////////
/// Init based on the installed sdf_format.xml file
bool init( SDFPtr _sdf )
{
  std::string filename = gazebo::common::SystemPaths::FindFileWithGazeboPaths("/sdf/gazebo.sdf");

  FILE *ftest = fopen(filename.c_str(), "r");
  if (ftest && initFile(filename, _sdf))
    return true;
  return false;
}

////////////////////////////////////////////////////////////////////////////////
bool initFile(const std::string &_filename, SDFPtr _sdf)
{
  std::string filename = gazebo::common::SystemPaths::FindFileWithGazeboPaths(_filename);

  TiXmlDocument xmlDoc;
  if (xmlDoc.LoadFile(filename))
  {
    return initDoc(&xmlDoc, _sdf);
  }
  else
    gzerr << "Unable to load file[" << _filename << "]\n";

  return false;
}

////////////////////////////////////////////////////////////////////////////////
bool initFile(const std::string &_filename, ElementPtr _sdf)
{
  std::string filename = gazebo::common::SystemPaths::FindFileWithGazeboPaths(_filename);

  TiXmlDocument xmlDoc;
  if (xmlDoc.LoadFile(filename))
    return initDoc(&xmlDoc, _sdf);
  else
    gzerr << "Unable to load file[" << _filename << "]\n";

  return false;
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
bool initDoc(TiXmlDocument *_xmlDoc, ElementPtr &_sdf)
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

  return initXml(xml, _sdf);
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
    const char *copyDataString = child->Attribute("copy_data");
    if (copyDataString && 
        (std::string(copyDataString) == "true" ||
         std::string(copyDataString) == "1"))
    {
      _sdf->SetCopyChildren(true);
    }
    else
    {
      ElementPtr element(new Element);
      initXml(child, element);
      _sdf->elementDescriptions.push_back(element);
    }
  }

  // Get all include elements
  for (TiXmlElement *child = _xml->FirstChildElement("include"); 
      child; child = child->NextSiblingElement("include"))
  {
    std::string filename = gazebo::common::SystemPaths::FindFileWithGazeboPaths(std::string("/sdf/") + child->Attribute("filename"));

    ElementPtr element(new Element);

    initFile(filename, element);
    _sdf->elementDescriptions.push_back(element);
  }

  return true;
}


////////////////////////////////////////////////////////////////////////////////
bool readFile(const std::string &_filename, SDFPtr &_sdf)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);
  if (readDoc(&xmlDoc, _sdf))
    return true;
  else
  {
    gzwarn << "parse as sdf version 1.0 failed, trying to parse as old deprecated format\n";
    if (deprecated_sdf::initWorldFile(_filename,_sdf))
      return true;
    else
    {
      gzwarn << "parse as old deprecated world file failed, trying old model format.\n";
      if (deprecated_sdf::initModelFile(_filename,_sdf->root))
        return true;
      else
      {
        gzerr << "parse as old deprecated model file failed.\n";
        return false;
      }
    }
  }
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
    gzwarn << "Could not parse the xml\n";
    return false;
  }

  /* check gazebo version, use old parser if necessary */
  TiXmlElement* gazebo_node = _xmlDoc->FirstChildElement("gazebo");
  if (gazebo_node &&
      gazebo_node->Attribute("version") &&
      (strcmp(gazebo_node->Attribute("version") , "1.0") == 0) )
  {
    /* parse new sdf xml */
    TiXmlElement* elemXml = _xmlDoc->FirstChildElement(_sdf->root->GetName());
    if (!readXml( elemXml, _sdf->root))
    {
      gzwarn << "Unable to parse sdf element[" << _sdf->root->GetName() << "]\n";
      return false;
    }
  }
  else
  {
    // try to use the old deprecated parser
    if (!gazebo_node)
      gzwarn << "Gazebo SDF has no gazebo element\n";
    else if (!gazebo_node->Attribute("version"))
      gzwarn << "Gazebo SDF gazebo element has no version\n";
    else if (strcmp(gazebo_node->Attribute("version") , "1.0") != 0)
      gzwarn << "Gazebo SDF version ["
            << gazebo_node->Attribute("version")
            << "] is not 1.0\n";
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

  Param_V::iterator iter;
  TiXmlAttribute *attribute = _xml->FirstAttribute();
  
  // Iterate over all the attributes defined in the give XML element
  while (attribute)
  {
    // Find the matching attribute in SDF
    for (iter = _sdf->attributes.begin(); 
         iter != _sdf->attributes.end(); iter++)
    {
      if ( (*iter)->GetKey() == attribute->Name() )
      {
        // Set the value of the SDF attribute
        if ( !(*iter)->SetFromString( attribute->ValueStr() ) )
        {
          gzerr << "Unable to read attribute[" << (*iter)->GetKey() << "]\n";
          return false;
        }
        break;
      }
    }

    if (iter == _sdf->attributes.end())
      gzwarn << "XML Attribute[" << attribute->Name() << "] in element[" << _xml->Value() << "] not defined in SDF, ignoring.\n";

    attribute = attribute->Next();
  }

  // Check that all required attributes have been set
  for (iter = _sdf->attributes.begin(); 
       iter != _sdf->attributes.end(); iter++)
  {
    if ((*iter)->GetRequired() && !(*iter)->GetSet())
    {
      gzerr << "Required attribute[" << (*iter)->GetKey() << "] in element[" << _xml->Value() << "] is not specified in SDF.\n";
    }
  }

  if ( _sdf->GetCopyChildren() )
  {
    copyChildren(_sdf, _xml);
  }
  else
  {
    // Iterate over all the child elements
    TiXmlElement* elemXml = NULL;
    for (elemXml = _xml->FirstChildElement(); elemXml; 
         elemXml = elemXml->NextSiblingElement())
    {

      if (std::string("include") == elemXml->Value())
      {
        std::string filename = gazebo::common::SystemPaths::FindFileWithGazeboPaths(std::string("models/") + elemXml->Attribute("filename"));
        SDFPtr includeSDF(new SDF); 
        init(includeSDF);

        readFile(filename, includeSDF);

        includeSDF->root->GetFirstElement()->SetParent( _sdf );
        _sdf->elements.push_back(includeSDF->root->GetFirstElement());

        //includeSDF.reset();
        continue;
      }

      // Find the matching element in SDF
      ElementPtr_V::iterator eiter;
      for (eiter = _sdf->elementDescriptions.begin(); 
          eiter != _sdf->elementDescriptions.end(); eiter++)
      {
        if ((*eiter)->GetName() == elemXml->Value())
        {
          ElementPtr element = (*eiter)->Clone();
          element->SetParent( _sdf );
          if (readXml( elemXml, element ) )
            _sdf->elements.push_back(element);
          else
            gzerr << "Error reading element\n";
          break;
        }
      }

      if (eiter == _sdf->elementDescriptions.end())
      {
        gzerr << "XML Element[" << elemXml->Value() 
          << "], child of element[" << _xml->Value() 
          << "] not defined in SDF. Ignoring.[" << _sdf->GetName() << "]\n";
      }
    }

    // Chek that all required elements have been set
    ElementPtr_V::iterator eiter;
    for (eiter = _sdf->elementDescriptions.begin(); 
        eiter != _sdf->elementDescriptions.end(); eiter++)
    {
      if ( (*eiter)->GetRequired() == "1" || (*eiter)->GetRequired() == "+")
      {
        if (!_sdf->HasElement( (*eiter)->GetName() ))
          gzerr << "XML Missing required element[" << (*eiter)->GetName() 
            << "], child of element[" << _sdf->GetName() << "]\n";
      }
    }
  }

  return true;
}

void copyChildren( ElementPtr &_sdf, TiXmlElement *_xml)
{
  // Iterate over all the child elements
  TiXmlElement* elemXml = NULL;
  for (elemXml = _xml->FirstChildElement(); elemXml; 
       elemXml = elemXml->NextSiblingElement())
  {
    ElementPtr element(new Element);
    element->SetParent( _sdf );
    element->SetName( elemXml->ValueStr() );
    element->AddValue("string", elemXml->GetText(), "1");

    _sdf->elements.push_back( element );
  }
}

}
