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

#ifndef URDF_PARSER_DEPRECATED_HH
#define URDF_PARSER_DEPRECATED_HH

#include <libxml/parser.h>
#include <boost/shared_ptr.hpp>

#include <string>
#include <map>

#include "sdf/interface/SDF.hh"
#include "math/Pose.hh"

namespace deprecated_sdf
{
  bool initLight(xmlNodePtr _config, sdf::ElementPtr _sdf);
  bool initSensor(xmlNodePtr _config, sdf::ElementPtr _sdf);
  bool initCamera(xmlNodePtr _config, sdf::ElementPtr _sdf);
  bool initRay(xmlNodePtr _config, sdf::ElementPtr _sdf);
  bool initContact(xmlNodePtr _config, sdf::ElementPtr _sdf);
  bool initInertial(xmlNodePtr _config, sdf::ElementPtr _sdf);
  bool initCollision(xmlNodePtr _config, sdf::ElementPtr _sdf);
  bool initOrigin(xmlNodePtr _config, sdf::ElementPtr _sdf);
  bool initLink(xmlNodePtr _config, sdf::ElementPtr _sdf);
  bool initVisual(xmlNodePtr _config, sdf::ElementPtr _sdf);
  bool initJoint(xmlNodePtr _config, sdf::ElementPtr _sdf);
  bool initModel(xmlNodePtr _config, sdf::ElementPtr _sdf);

  /// \brief Load Model given a filename
  bool initModelFile(const std::string &_filename, sdf::SDFPtr &_sdf);

  /// \brief Load Model from a XML-string
  bool initModelString(const std::string &_xmlstring, sdf::SDFPtr &_sdf);

  /// \brief Load Model from TiXMLDocument
  bool initModelDoc(xmlDocPtr _xml, sdf::SDFPtr &_sdf);

  /// \brief Load Model from TiXMLElement
  bool initModelXml(xmlNodePtr _xml, sdf::SDFPtr &_sdf);


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
  bool initScene(xmlNodePtr _config, sdf::ElementPtr _sdf);

  /// physics
  bool initPhysics(xmlNodePtr _config, sdf::ElementPtr _sdf);

  /// copying <controller:...> to <plugins>
  void copyBlockChildren(xmlNodePtr _config, sdf::ElementPtr _sdf);
  bool controller2Plugins(xmlNodePtr pluginXml, sdf::ElementPtr _sdf);
  bool getProjectors(xmlNodePtr _config, sdf::ElementPtr _sdf);
  bool getGrippers(xmlNodePtr _config, sdf::ElementPtr _sdf);

  bool initAttr(xmlNodePtr _node, const std::string &_key, sdf::ParamPtr _attr);
  bool initElem(xmlNodePtr _node, const std::string &_key, sdf::ParamPtr _attr);

  ////////////////////////////////////////////////////////////////////////////
  //
  // Some helper functions copied from old XMLConfigNode class
  //
  // Get a child based on a name. Returns null if not found
  xmlNodePtr firstChildElement(xmlDocPtr node, const std::string &name);
  xmlNodePtr firstChildElement(xmlNodePtr node, const std::string &name);
  xmlNodePtr nextSiblingElement(xmlNodePtr node, const std::string &name);

  ////////////////////////////////////////////////////////////////////////////
  // Get the next sibling of node according the namespace prefix
  xmlNodePtr getNextByNSPrefix(xmlNodePtr node, const std::string &prefix);
  ////////////////////////////////////////////////////////////////////////////
  // Get the first child with the appropriate NS prefix
  xmlNodePtr getChildByNSPrefix(xmlNodePtr node, const std::string &prefix);

  ////////////////////////////////////////////////////////////////////////////
  // Get a value associated with a node.
  std::string getNodeValue(xmlNodePtr node, const std::string &key);
  std::string getNodeTuple(xmlNodePtr node, const std::string &key, int index);

  ////////////////////////////////////////////////////////////////////////////
  // Get the value of this node
  std::string getValue(xmlNodePtr node);
  void PreParser(const std::string &fname, std::string &output);
  void ExpandIncludes(xmlNodePtr _xml);
  std::string ToString(xmlNodePtr _xml);
  std::string ToString(xmlDocPtr doc);
}
#endif

