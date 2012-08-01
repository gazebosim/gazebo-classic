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

#include "gazebo/sdf/interface/Converter.hh"
#include "gazebo/sdf/interface/SDF.hh"
#include "gazebo/sdf/interface/Param.hh"
#include "gazebo/sdf/interface/parser.hh"
#include "gazebo/sdf/interface/parser_deprecated.hh"

#include "gazebo/common/Console.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/common/Common.hh"

namespace sdf
{
//////////////////////////////////////////////////
bool init(SDFPtr _sdf)
{
  bool result = false;
  std::string filename = gazebo::common::find_file("gazebo.sdf");

  FILE *ftest = fopen(filename.c_str(), "r");
  if (ftest && initFile(filename, _sdf))
  {
    result = true;
    fclose(ftest);
  }

  return result;
}

//////////////////////////////////////////////////
bool initFile(const std::string &_filename, SDFPtr _sdf)
{
  std::string filename = gazebo::common::find_file(_filename);

  TiXmlDocument xmlDoc;
  if (xmlDoc.LoadFile(filename))
  {
    return initDoc(&xmlDoc, _sdf);
  }
  else
    gzerr << "Unable to load file[" << _filename << "]\n";

  return false;
}

//////////////////////////////////////////////////
bool initFile(const std::string &_filename, ElementPtr _sdf)
{
  std::string filename = gazebo::common::find_file(_filename);

  TiXmlDocument xmlDoc;
  if (xmlDoc.LoadFile(filename))
    return initDoc(&xmlDoc, _sdf);
  else
    gzerr << "Unable to load file[" << filename << "]\n";

  return false;
}

//////////////////////////////////////////////////
bool initString(const std::string &_xmlString, SDFPtr _sdf)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());

  return initDoc(&xmlDoc, _sdf);
}

//////////////////////////////////////////////////
bool initDoc(TiXmlDocument *_xmlDoc, SDFPtr _sdf)
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

//////////////////////////////////////////////////
bool initDoc(TiXmlDocument *_xmlDoc, ElementPtr _sdf)
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

//////////////////////////////////////////////////
bool initXml(TiXmlElement *_xml, ElementPtr _sdf)
{
  const char *nameString = _xml->Attribute("name");
  if (!nameString)
  {
    gzerr << "Element is missing the name attribute\n";
    return false;
  }
  _sdf->SetName(std::string(nameString));

  const char *requiredString = _xml->Attribute("required");
  if (!requiredString)
  {
    gzerr << "Element is missing the required attributed\n";
    return false;
  }
  _sdf->SetRequired(requiredString);

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
    requiredString = child->Attribute("required");

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
    std::string requiredStr = requiredString;
    boost::trim(requiredStr);
    bool required = requiredStr == "1" ? true : false;

    _sdf->AddAttribute(name, type, defaultValue, required);
  }

  // Read the element description
  TiXmlElement *descChild = _xml->FirstChildElement("description");
  if (descChild)
  {
    _sdf->SetDescription(descChild->GetText());
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
      _sdf->AddElementDescription(element);
    }
  }

  // Get all include elements
  for (TiXmlElement *child = _xml->FirstChildElement("include");
      child; child = child->NextSiblingElement("include"))
  {
    std::string filename = child->Attribute("filename");

    ElementPtr element(new Element);

    initFile(filename, element);
    _sdf->AddElementDescription(element);
  }

  return true;
}


//////////////////////////////////////////////////
bool readFile(const std::string &_filename, SDFPtr _sdf)
{
  TiXmlDocument xmlDoc;
  std::string filename = gazebo::common::find_file(_filename);

  xmlDoc.LoadFile(filename);
  if (readDoc(&xmlDoc, _sdf, filename))
    return true;
  else
  {
    if (deprecated_sdf::initWorldFile(filename, _sdf))
    {
      gzwarn << "DEPRECATED GAZEBO WORLD FILE\n"
             << "On July 1st, 2012, this formate will no longer by supported\n"
             << "Convert your files using the gzsdf command line tool\n"
             << "You have been warned!\n\n";
      return true;
    }
    else
    {
      if (deprecated_sdf::initModelFile(filename, _sdf))
      {
        gzwarn << "DEPRECATED GAZEBO MODEL FILE\n"
          << "On July 1st, 2012, this formate will no longer by supported\n"
          << "Convert your files using the gzsdf command line tool\n"
          << "You have been warned!\n\n";
        return true;
      }
      else
      {
        gzerr << "parse as old deprecated model file failed.\n";
        return false;
      }
    }
  }
}

//////////////////////////////////////////////////
bool readString(const std::string &_xmlString, SDFPtr _sdf)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());
  if (readDoc(&xmlDoc, _sdf, "data-string"))
    return true;
  else
  {
    if (deprecated_sdf::initWorldString(_xmlString, _sdf))
    {
      gzwarn << "DEPRECATED GAZEBO WORLD FILE\n"
             << "On July 1st, 2012, this formate will no longer by supported\n"
             << "Convert your files using the gzsdf command line tool\n"
             << "You have been warned!\n\n";
      return true;
    }
    else
    {
      if (deprecated_sdf::initModelString(_xmlString, _sdf))
      {
        gzwarn << "DEPRECATED GAZEBO MODEL FILE\n"
          << "On July 1st, 2012, this formate will no longer by supported\n"
          << "Convert your files using the gzsdf command line tool\n"
          << "You have been warned!\n\n";
        return true;
      }
      else
      {
        gzerr << "parse as old deprecated model file failed.\n";
        return false;
      }
    }
  }
}

//////////////////////////////////////////////////
bool readString(const std::string &_xmlString, ElementPtr _sdf)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());
  if (readDoc(&xmlDoc, _sdf, "data-string"))
    return true;
  else
  {
    gzerr << "parse as sdf version " << SDF_VERSION << " failed, "
          << "should try to parse as old deprecated format\n";
    return false;
  }
}

//////////////////////////////////////////////////
bool readDoc(TiXmlDocument *_xmlDoc, SDFPtr _sdf, const std::string &_source)
{
  if (!_xmlDoc)
  {
    gzwarn << "Could not parse the xml from source[" << _source << "]\n";
    return false;
  }

  /* check gazebo version, use old parser if necessary */
  TiXmlElement* gazeboNode = _xmlDoc->FirstChildElement("gazebo");

  if (gazeboNode && gazeboNode->Attribute("version"))
  {
    if (strcmp(gazeboNode->Attribute("version"), SDF_VERSION) != 0)
    {
      gzwarn << "Converting a deprecatd SDF source[" << _source << "].\n"; 
      Converter::Convert(gazeboNode, SDF_VERSION);
    }

    /* parse new sdf xml */
    TiXmlElement* elemXml = _xmlDoc->FirstChildElement(_sdf->root->GetName());
    if (!readXml(elemXml, _sdf->root))
    {
      gzerr << "Unable to parse sdf element[" << _sdf->root->GetName() << "]\n";
      return false;
    }
  }
  else
  {
    // try to use the old deprecated parser
    if (!gazeboNode)
      gzwarn << "Gazebo SDF has no gazebo element\n";
    else if (!gazeboNode->Attribute("version"))
      gzwarn << "Gazebo SDF gazebo element has no version\n";
    else if (strcmp(gazeboNode->Attribute("version"), SDF_VERSION) != 0)
      gzwarn << "Gazebo SDF version ["
            << gazeboNode->Attribute("version")
            << "] is not " << SDF_VERSION << "\n";
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool readDoc(TiXmlDocument *_xmlDoc, ElementPtr _sdf,
             const std::string &_source)
{
  if (!_xmlDoc)
  {
    gzwarn << "Could not parse the xml\n";
    return false;
  }

  /* check gazebo version, use old parser if necessary */
  TiXmlElement* gazeboNode = _xmlDoc->FirstChildElement("gazebo");
  if (gazeboNode && gazeboNode->Attribute("version"))
  {
    if (strcmp(gazeboNode->Attribute("version"), SDF_VERSION) != 0)
    {
      gzwarn << "Converting a deprecatd SDF source[" << _source << "].\n"; 
      Converter::Convert(gazeboNode, SDF_VERSION);
    }

    TiXmlElement* elemXml = gazeboNode;
    if (gazeboNode->Value() != _sdf->GetName() &&
        gazeboNode->FirstChildElement(_sdf->GetName()))
    {
      elemXml = gazeboNode->FirstChildElement(_sdf->GetName());
    }

    /* parse new sdf xml */
    if (!readXml(elemXml, _sdf))
    {
      gzwarn << "Unable to parse sdf element["
             << _sdf->GetName() << "]\n";
      return false;
    }
  }
  else
  {
    // try to use the old deprecated parser
    if (!gazeboNode)
      gzwarn << "Gazebo SDF has no gazebo element\n";
    else if (!gazeboNode->Attribute("version"))
      gzwarn << "Gazebo SDF gazebo element has no version\n";
    else if (strcmp(gazeboNode->Attribute("version"), SDF_VERSION) != 0)
      gzwarn << "Gazebo SDF version ["
            << gazeboNode->Attribute("version")
            << "] is not " << SDF_VERSION << "\n";
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool readXml(TiXmlElement *_xml, ElementPtr _sdf)
{
  if (_sdf->GetRequired() == "-1")
  {
    gzwarn << "SDF Element[" << _sdf->GetName() << "] is deprecated\n";
    return true;
  }

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

  if (_xml->GetText() != NULL && _sdf->GetValue())
  {
    _sdf->GetValue()->SetFromString(_xml->GetText());
  }

  TiXmlAttribute *attribute = _xml->FirstAttribute();

  unsigned int i = 0;

  // Iterate over all the attributes defined in the give XML element
  while (attribute)
  {
    // Find the matching attribute in SDF
    for (i = 0; i < _sdf->GetAttributeCount(); ++i)
    {
      ParamPtr p = _sdf->GetAttribute(i);
      if (p->GetKey() == attribute->Name())
      {
        // Set the value of the SDF attribute
        if (!p->SetFromString(attribute->ValueStr()))
        {
          gzerr << "Unable to read attribute[" << p->GetKey() << "]\n";
          return false;
        }
        break;
      }
    }

    if (i == _sdf->GetAttributeCount())
    {
      gzwarn << "XML Attribute[" << attribute->Name()
             << "] in element[" << _xml->Value()
             << "] not defined in SDF, ignoring.\n";
    }

    attribute = attribute->Next();
  }

  // Check that all required attributes have been set
  for (i = 0; i < _sdf->GetAttributeCount(); ++i)
  {
    ParamPtr p = _sdf->GetAttribute(i);
    if (p->GetRequired() && !p->GetSet())
    {
      gzerr << "Required attribute[" << p->GetKey()
        << "] in element[" << _xml->Value() << "] is not specified in SDF.\n";
      return false;
    }
  }

  if (_sdf->GetCopyChildren())
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
        std::string filename = gazebo::common::find_file(
              std::string("models/") + elemXml->Attribute("filename"));

        SDFPtr includeSDF(new SDF);
        init(includeSDF);

        if (!readFile(filename, includeSDF))
        {
          gzerr << "Unable to read file[" << filename << "]\n";
          return false;
        }

        if (elemXml->Attribute("model_name"))
          includeSDF->root->GetElement("model")->GetAttribute(
              "name")->SetFromString(elemXml->Attribute("model_name"));
        if (elemXml->Attribute("model_pose"))
          includeSDF->root->GetElement("model")->GetOrCreateElement(
              "origin")->GetAttribute("pose")->SetFromString(
                elemXml->Attribute("model_pose"));

        for (TiXmlElement *childElemXml = elemXml->FirstChildElement();
             childElemXml; childElemXml = childElemXml->NextSiblingElement())
        {
          if (std::string("plugin") == childElemXml->Value())
          {
            sdf::ElementPtr pluginElem;
            pluginElem = includeSDF->root->GetElement(
                "model")->AddElement("plugin");

            pluginElem->GetAttribute("filename")->SetFromString(
                childElemXml->Attribute("filename"));
            pluginElem->GetAttribute("name")->SetFromString(
                childElemXml->Attribute("name"));
          }
        }

        if (_sdf->GetName() == "model")
        {
          addNestedModel(_sdf, includeSDF->root);
        }
        else
        {
          includeSDF->root->GetFirstElement()->SetParent(_sdf);
          _sdf->InsertElement(includeSDF->root->GetFirstElement());
          includeSDF->root->GetFirstElement()->SetInclude(elemXml->Attribute(
                "filename"));
        }

        continue;
      }

      // Find the matching element in SDF
      unsigned int descCounter = 0;
      for (descCounter = 0;
           descCounter != _sdf->GetElementDescriptionCount(); ++descCounter)
      {
        ElementPtr elemDesc = _sdf->GetElementDescription(descCounter);
        if (elemDesc->GetName() == elemXml->Value())
        {
          ElementPtr element = elemDesc->Clone();
          element->SetParent(_sdf);
          if (readXml(elemXml, element))
            _sdf->InsertElement(element);
          else
          {
            gzerr << "Error reading element\n";
            return false;
          }
          break;
        }
      }

      if (descCounter == _sdf->GetElementDescriptionCount())
      {
        gzerr << "XML Element[" << elemXml->Value()
          << "], child of element[" << _xml->Value()
          << "] not defined in SDF. Ignoring.[" << _sdf->GetName() << "]\n";
        return false;
      }
    }

    // Chek that all required elements have been set
    unsigned int descCounter = 0;
    for (descCounter = 0;
         descCounter != _sdf->GetElementDescriptionCount(); ++descCounter)
    {
      ElementPtr elemDesc = _sdf->GetElementDescription(descCounter);

      if (elemDesc->GetRequired() == "1" || elemDesc->GetRequired() == "+")
      {
        if (!_sdf->HasElement(elemDesc->GetName()))
        {
          if (_sdf->GetName() == "joint" &&
              _sdf->GetValueString("type") != "ball")
          {
            gzerr << "XML Missing required element[" << elemDesc->GetName()
              << "], child of element[" << _sdf->GetName() << "]\n";
            return false;
          }
          else
          {
            // Add default element
            _sdf->AddElement(elemDesc->GetName());
          }
        }
      }
    }
  }

  return true;
}

/////////////////////////////////////////////////
void copyChildren(ElementPtr _sdf, TiXmlElement *_xml)
{
  // Iterate over all the child elements
  TiXmlElement* elemXml = NULL;
  for (elemXml = _xml->FirstChildElement(); elemXml;
       elemXml = elemXml->NextSiblingElement())
  {
    std::string elem_name = elemXml->ValueStr();
    if (_sdf->HasElementDescription(elem_name))
    {
      sdf::ElementPtr element = _sdf->AddElement(elem_name);

      // FIXME: copy attributes
      for (TiXmlAttribute *attribute = elemXml->FirstAttribute();
           attribute; attribute = attribute->Next())
      {
        element->GetAttribute(attribute->Name())->SetFromString(
          attribute->ValueStr());
      }

      // copy value
      std::string value = elemXml->GetText();
      if (!value.empty())
          element->GetValue()->SetFromString(value);
    }
    else
    {
      ElementPtr element(new Element);
      element->SetParent(_sdf);
      element->SetName(elem_name);
      if (elemXml->GetText() != NULL)
        element->AddValue("string", elemXml->GetText(), "1");
      else
        gzerr << "trying to copy stuff inside <plugin> block, "
              << "but they have NULL contents\n";

      _sdf->InsertElement(element);
    }
  }
}

/////////////////////////////////////////////////
void addNestedModel(ElementPtr _sdf, ElementPtr _includeSDF)
{
  ElementPtr modelPtr = _includeSDF->GetElement("model");
  ElementPtr elem = modelPtr->GetFirstElement();
  std::map<std::string, std::string> replace;

  gazebo::math::Pose modelPose =
    modelPtr->GetOrCreateElement("origin")->GetValuePose("pose");

  std::string modelName = modelPtr->GetValueString("name");
  while (elem)
  {
    if (elem->GetName() == "link")
    {
      std::string elemName = elem->GetValueString("name");
      std::string newName =  modelName + "::" + elemName;
      replace[elemName] = newName;
      if (elem->HasElementDescription("origin"))
      {
        ElementPtr originElem = elem->GetOrCreateElement("origin");
        gazebo::math::Pose newPose = gazebo::math::Pose(
          modelPose.pos +
            modelPose.rot.RotateVector(originElem->GetValuePose("pose").pos),
            modelPose.rot * originElem->GetValuePose("pose").rot);
        originElem->GetAttribute("pose")->Set(newPose);
      }
    }
    else if (elem->GetName() == "joint")
    {
      // for joints, we need to
      //   prefix name like we did with links, and
      std::string elemName = elem->GetValueString("name");
      std::string newName =  modelName + "::" + elemName;
      replace[elemName] = newName;
      //   rotate the joint axis because they are model-global
      if (elem->HasElement("axis"))
      {
        ElementPtr axisElem = elem->GetElement("axis");
        gazebo::math::Vector3 newAxis =  modelPose.rot.RotateVector(
          axisElem->GetValueVector3("xyz"));
        axisElem->GetAttribute("xyz")->Set(newAxis);
      }
    }
    elem = elem->GetNextElement();
  }

  std::string str = _includeSDF->ToString("");
  for (std::map<std::string, std::string>::iterator iter = replace.begin();
       iter != replace.end(); ++iter)
  {
    boost::replace_all(str, std::string("\"")+iter->first + "\"",
                       std::string("\"") + iter->second + "\"");
    boost::replace_all(str, std::string("'")+iter->first + "'",
                       std::string("'") + iter->second + "'");
    boost::replace_all(str, std::string(">")+iter->first + "<",
                       std::string(">") + iter->second + "<");
  }

  _includeSDF->ClearElements();
  readString(str, _includeSDF);

  elem = _includeSDF->GetElement("model")->GetFirstElement();
  ElementPtr nextElem;
  while (elem)
  {
    nextElem = elem->GetNextElement();

    if (elem->GetName() != "origin")
    {
      elem->SetParent(_sdf);
      _sdf->InsertElement(elem);
    }
    elem = nextElem;
  }
}
}
