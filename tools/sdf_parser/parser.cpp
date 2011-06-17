/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <tinyxml.h>
#include <stdio.h>

#include "parser.h"

namespace sdf
{

bool getBoolFromStr(std::string _str, bool &_value)
{
  boost::to_lower(_str);
  if (_str == "true" || _str == "t" || _str == "1")
    _value = true;
  else if (_str == "false" || _str == "f" || _str == "0")
    _value = false;
  else
  {
    _value = false;
    return false;
  }

  return true;
}

bool getDoubleFromStr(const std::string &_str, double &_value)
{
  try
  {
    _value = boost::lexical_cast<double>(_str);
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("ERR: string is not a double format\n");
    return false;
  }

  return true;
}

bool getIntFromStr(const std::string &_str, int &_value)
{
  try
  {
    _value = boost::lexical_cast<int>(_str);
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("ERR: string is not an int format\n");
    return false;
  }

  return true;
}

bool getUIntFromStr(const std::string &_str, unsigned int &_value)
{
  try
  {
    _value = boost::lexical_cast<unsigned int>(_str);
  }
  catch (boost::bad_lexical_cast &e)
  {
    printf("ERR: string is not an unsigned int format\n");
    return false;
  }

  return true;
}

bool getPlugins(TiXmlElement *_parentXml, 
                std::map<std::string, boost::shared_ptr<Plugin> > &_plugins)
{
  // Get all plugins 
  for (TiXmlElement* pluginXml = _parentXml->FirstChildElement("plugin"); 
      pluginXml; pluginXml = _parentXml->NextSiblingElement("plugin"))
  {
    boost::shared_ptr<Plugin> plugin;
    plugin.reset(new Plugin);

    if (plugin->InitXml(pluginXml))
    {
      if (_plugins.find(plugin->name) != _plugins.end())
      {
        printf("plugin '%s' is not unique.\n", plugin->name.c_str());
        plugin.reset();
        return false;
      }
      else
      {
        _plugins.insert(make_pair(plugin->name,plugin));
        printf("successfully added a new plugin '%s'\n", plugin->name.c_str());
      }
    }
    else
    {
      printf("plugin xml is not initialized correctly\n");
      plugin.reset();
      return false;
    }
  }

  return true;
}


}
