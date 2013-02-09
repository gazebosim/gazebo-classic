/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "gazebo/common/Common.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/sdf/interface/Converter.hh"

using namespace sdf;

/////////////////////////////////////////////////
bool Converter::Convert(TiXmlDocument *_doc, const std::string &_toVersion,
                        bool _quiet)
{
  TiXmlElement *elem = _doc->FirstChildElement("gazebo");

  // Replace <gazebo> with <sdf>
  if (elem && boost::lexical_cast<double>(_toVersion) >= 1.3)
  {
    elem->SetValue("sdf");
    std::cout << "Set SDF value\n";
  }
  else if (!elem)
    elem = _doc->FirstChildElement("sdf");

  if (!elem || !elem->Attribute("version"))
  {
    gzerr << "  Unable to determine original SDF version\n";
    return false;
  }

  std::string origVersion = elem->Attribute("version");

  if (origVersion == _toVersion)
    return true;

  if (!_quiet)
  {
    std::cout << gzclr_start(33)
              << "  Version[" << origVersion << "] to Version[" << _toVersion
              << "]\n"
              << "  Please use the gzsdf tool to update your SDF files.\n"
              << "    $ gzsdf convert [sdf_file]\n"
              << gzclr_end;
  }

  elem->SetAttribute("version", _toVersion);

  boost::replace_all(origVersion, ".", "_");

  std::string filename = gazebo::common::find_file(
      std::string("sdf/") + _toVersion + "/" + origVersion + ".convert");

  TiXmlDocument xmlDoc;
  if (!xmlDoc.LoadFile(filename))
  {
    gzerr << "Unable to load file[" << filename << "]\n";
    return false;
  }

  ConvertImpl(elem, xmlDoc.FirstChildElement("convert"));

  return true;
}

/////////////////////////////////////////////////
void Converter::ConvertImpl(TiXmlElement *_elem, TiXmlElement *_convert)
{
  CheckDeprecation(_elem, _convert);

  for (TiXmlElement *convertElem = _convert->FirstChildElement("convert");
       convertElem; convertElem = convertElem->NextSiblingElement("convert"))
  {
    TiXmlElement *elem = NULL;
    elem = _elem->FirstChildElement(convertElem->Attribute("name"));
    while (elem)
    {
      ConvertImpl(elem, convertElem);
      elem = elem->NextSiblingElement(convertElem->Attribute("name"));
    }
  }

  for (TiXmlElement *renameElem = _convert->FirstChildElement("rename");
       renameElem; renameElem = renameElem->NextSiblingElement("rename"))
  {
    TiXmlElement *fromConvertElem = renameElem->FirstChildElement("from");
    TiXmlElement *toConvertElem = renameElem->FirstChildElement("to");

    const char *fromElemName = fromConvertElem->Attribute("element");
    const char *fromAttrName = fromConvertElem->Attribute("attribute");

    const char *toElemName = toConvertElem->Attribute("element");
    const char *toAttrName = toConvertElem->Attribute("attribute");

    const char *value = GetValue(fromElemName, fromAttrName, _elem);
    if (!value)
      continue;

    if (!toElemName)
    {
      gzerr << "No 'to' element name specified\n";
      continue;
    }

    TiXmlElement *replaceTo = new TiXmlElement(toElemName);
    if (toAttrName)
      replaceTo->SetAttribute(toAttrName, value);
    else
    {
      TiXmlText *text = new TiXmlText(value);
      replaceTo->LinkEndChild(text);
    }

    if (fromElemName)
    {
      TiXmlElement *replaceFrom = _elem->FirstChildElement(fromElemName);
      _elem->ReplaceChild(replaceFrom, *replaceTo);
    }
    else if (fromAttrName)
    {
      _elem->RemoveAttribute(fromAttrName);
      _elem->LinkEndChild(replaceTo);
    }
  }
}

/////////////////////////////////////////////////
const char *Converter::GetValue(const char *_valueElem, const char *_valueAttr,
                                TiXmlElement *_elem)
{
  if (_valueElem)
  {
    // Check to see if the element that is being converted has the value
    if (!_elem->FirstChildElement(_valueElem))
      return NULL;

    if (_valueAttr)
      return _elem->FirstChildElement(_valueElem)->Attribute(_valueAttr);
    else
      return _elem->FirstChildElement(_valueElem)->GetText();
  }
  else if (_valueAttr)
  {
    return _elem->Attribute(_valueAttr);
  }

  return NULL;
}

/////////////////////////////////////////////////
void Converter::CheckDeprecation(TiXmlElement *_elem, TiXmlElement *_convert)
{
  // Process deprecated elements
  for (TiXmlElement *deprecatedElem = _convert->FirstChildElement("deprecated");
      deprecatedElem;
      deprecatedElem = deprecatedElem->NextSiblingElement("deprecated"))
  {
    std::string value = deprecatedElem->GetText();
    std::vector<std::string> valueSplit;
    boost::split(valueSplit, value, boost::is_any_of("/"));

    bool found = false;
    TiXmlElement *e = _elem;
    std::ostringstream stream;

    std::string prefix = "";
    for (unsigned int i = 0; i < valueSplit.size() && !found; ++i)
    {
      if (e->FirstChildElement(valueSplit[i]))
      {
        if (stream.str().size() != 0)
        {
          stream << ">\n";
          prefix += "  ";
        }

        stream << prefix << "<" << valueSplit[i];
        e = e->FirstChildElement(valueSplit[i]);
      }
      else if (e->Attribute(valueSplit[i]))
      {
        stream << " " << valueSplit[i] << "='"
               << e->Attribute(valueSplit[i].c_str()) << "'";
        found = true;
      }
    }

    gzwarn << "Deprecated SDF Values in original file:\n"
           << stream.str() << "\n\n";
  }
}
