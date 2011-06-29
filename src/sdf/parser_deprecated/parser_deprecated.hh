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

/* Author: Nate Koenig, John Hsu */

#ifndef URDF_PARSER_HH
#define URDF_PARSER_HH

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <libxml/parser.h>

#include "sdf/interface/SDF.hh"
#include "math/Pose.hh"

namespace deprecated_sdf
{
  bool initLight(xmlNodePtr _config, sdf::ElementPtr &_sdf);
  bool initSensor(xmlNodePtr _config, sdf::ElementPtr &_sdf);
  bool initCamera(xmlNodePtr _config, sdf::ElementPtr &_sdf);
  bool initRay(xmlNodePtr _config, sdf::ElementPtr &_sdf);
  bool initContact(xmlNodePtr _config, sdf::ElementPtr &_sdf);
  bool initInertial(xmlNodePtr _config, sdf::ElementPtr &_sdf);
  bool initCollision(xmlNodePtr _config, sdf::ElementPtr &_sdf);
  bool initOrigin(xmlNodePtr _config, sdf::ElementPtr &_sdf);
  bool initLink(xmlNodePtr _config, sdf::ElementPtr &_sdf);
  bool initVisual(xmlNodePtr _config, sdf::ElementPtr &_sdf);
  bool initJoint(xmlNodePtr _config, sdf::ElementPtr &_sdf);
  bool initModel(xmlNodePtr _config, sdf::ElementPtr &_sdf);

  /// \brief Load Model given a filename
  bool initModelFile(const std::string &_filename, sdf::ElementPtr &_sdf);

 /// \brief Load Model from a XML-string
  bool initModelString(const std::string &_xmlstring, sdf::ElementPtr &_sdf);

  /// \brief Load Model from TiXMLDocument
  bool initModelDoc(xmlDocPtr _xml, sdf::ElementPtr &_sdf);

  /// \brief Load Model from TiXMLElement
  bool initModelXml(xmlNodePtr _xml, sdf::ElementPtr &_sdf);


  /// \brief Load world given a filename
  bool initWorldFile(const std::string &_filename, sdf::SDFPtr &_sdf);

  /// \brief Load world from a XML-string
  bool initWorldString(const std::string &_xmlstring, 
                       sdf::SDFPtr &_sdf);

   /// \brief Load World from TiXMLDocument
  bool initWorldDoc(xmlDocPtr _xml, sdf::SDFPtr &_sdf);

   /// \brief Load Model from TiXMLElement
  bool initWorld(xmlNodePtr _xml, sdf::SDFPtr &_sdf);

  /// scene
  bool initScene(xmlNodePtr _config, sdf::ElementPtr &_sdf);
  
  /// physics
  bool initPhysics(xmlNodePtr _config, sdf::ElementPtr &_sdf);

  bool getPlugins(xmlNodePtr pluginXml, std::map<std::string, 
                  sdf::ElementPtr > &_sdf);

  bool initAttr(xmlNodePtr _node, const std::string _key, sdf::ParamPtr _attr);

  ////////////////////////////////////////////////////////////////////////////
  //
  // Some helper functions copied from old XMLConfigNode class
  //
  // Get a child based on a name. Returns null if not found
  xmlNodePtr firstChildElement( xmlDocPtr node, const std::string &name)
  {
    xmlNodePtr tmp;
    for (tmp = node->xmlChildrenNode; tmp != NULL; tmp = tmp->next )
      if (tmp->name && name == (const char*)tmp->name) break;

    return tmp;
  }
  xmlNodePtr firstChildElement( xmlNodePtr node, const std::string &name)
  {
    xmlNodePtr tmp;
    for (tmp = xmlFirstElementChild(node); tmp != NULL; tmp = xmlNextElementSibling(tmp) )
      if (tmp->name && (name == (const char*)tmp->name)) break;

    return tmp;
  }

  xmlNodePtr nextSiblingElement( xmlNodePtr node, const std::string &name)
  {
    xmlNodePtr tmp;
    for (tmp = xmlNextElementSibling(node); tmp != NULL; tmp = xmlNextElementSibling(tmp) )
      if (tmp->name && (name == (const char*)tmp->name)) break;

    return tmp;
  }

  ////////////////////////////////////////////////////////////////////////////
  // Get the next sibling of node according the namespace prefix
  xmlNodePtr getNextByNSPrefix(xmlNodePtr node, const std::string &prefix)
  {
    xmlNodePtr tmp;
    for (tmp = xmlNextElementSibling(node); tmp != NULL; tmp = xmlNextElementSibling(tmp) )
      if (tmp->ns && prefix == (const char*)tmp->ns->prefix)
        break;
    return tmp;
  }
  ////////////////////////////////////////////////////////////////////////////
  // Get the first child with the appropriate NS prefix
  xmlNodePtr getChildByNSPrefix(xmlNodePtr node, const std::string &prefix )
  {
    xmlNodePtr tmp;
    for (tmp = node->xmlChildrenNode; tmp != NULL; tmp = xmlNextElementSibling(tmp) )
      if (tmp->ns && prefix == (const char*)tmp->ns->prefix)
        break;
    return tmp;
  }

  ////////////////////////////////////////////////////////////////////////////
  // Get a value associated with a node.
  std::string getNodeValue( xmlNodePtr node, const std::string &key )
  {
    std::string result;
    xmlChar *value=NULL;

    // First check if the key is an attribute
    if (xmlHasProp( node, (xmlChar*) key.c_str() ))
    {
      value = xmlGetProp( node, (xmlChar*) key.c_str() );

      // If not an attribute, then it should be a child node
    }
    else if (key == (const char*)node->name)
    {
      value = xmlNodeListGetString( node->doc, node->xmlChildrenNode, 1);
    }
    else
    {
      xmlNodePtr currNode;

      currNode = node->xmlChildrenNode;

      // Loop through children
      while (currNode)
      {
        // If the name matches, then return its value
        if (key == (const char*)currNode->name)
        {
          value = xmlNodeListGetString( node->doc, currNode->xmlChildrenNode, 1 );
          break;
        }

        currNode = currNode->next;
      }
    }

    if (value)
    {
      result = (char*)value;
      boost::trim(result);

      xmlFree(value);
    }

    return result;
  }

  std::string getNodeTuple(xmlNodePtr node, const std::string &key, int index)
  {
    std::string value;
    std::string nvalue;
    int i, a, b, state, count;

    value = getNodeValue(node, key);

    if (value.empty())
      return std::string();

    state = 0;
    count = 0;
    a = b = 0;

    for (i = 0; i < (int)value.size(); i++)
    {
      // Look for start of element
      if (state == 0)
      {
        if (!isspace( value[i] ))
        {
          a = i;
          state = 1;
        }
      }

      // Look for end of element
      else if (state == 1)
      {
        if (isspace( value[i] ))
        {
          state = 0;
          b = i - 1;
          count++;
          if (count > index)
            break;
        }
      }
    }
    if (state == 1)
    {
      b = i - 1;
      count++;
    }

    if (count == index + 1)
    {
      const char *s = value.c_str() + a;
      size_t size = b-a+2;
      char *end = (char *)memchr(s,0,size);

      if (end)
        size = end -s + 1;

      char *r = (char *)malloc(size);

      if (size)
      {
        memcpy(r, s, size-1);
        r[size-1] = '\0';
      }

      nvalue = r;
    }

    return nvalue;
  }




  ////////////////////////////////////////////////////////////////////////////
  // Get the value of this node
  std::string getValue(xmlNodePtr node)
  {
    const char *v =(const char*)xmlNodeListGetString(node->doc, node->xmlChildrenNode, 1); 
    if (v)
      return std::string(v);
    else
      return std::string();
  }


  void PreParser(const std::string &fname, std::string &output)
  {
    std::ifstream ifs(fname.c_str(), std::ios::in);
    std::string line;

    while (ifs.good())
    {
      std::getline(ifs, line);
      boost::trim(line);

      if (boost::find_first(line,"<include"))
      {
        int start = line.find("filename=");
        start += strlen("filename=") + 1;
        int end = line.find_first_of("'\"", start);
        std::string fname2 = line.substr(start, end-start);
        PreParser(fname2, output);
      }
      else
        output += line + "\n";
    }

    ifs.close();
  }


}

#endif
