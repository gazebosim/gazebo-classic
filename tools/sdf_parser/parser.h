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

#ifndef URDF_PARSER_H
#define URDF_PARSER_H

#include <string>
#include <map>
#include <tinyxml.h>
#include <boost/function.hpp>
#include "link.h"


namespace gdf{

class Parser
{
public:
  Parser();

  /// \brief Load Model from TiXMLElement
  bool init(TiXmlElement *xml);
  /// \brief Load Model from TiXMLDocument
  bool init(TiXmlDocument *xml);

  boost::shared_ptr<const Link> getLink(const std::string& name) const;
  boost::shared_ptr<const Joint> getJoint(const std::string& name) const;
  const std::string& getName() const {return name_;};

  void getLinks(std::vector<boost::shared_ptr<Link> >& links) const;

  /// \brief complete list of Links
  std::map<std::string, boost::shared_ptr<Link> > links_;
  /// \brief complete list of Joints
  std::map<std::string, boost::shared_ptr<Joint> > joints_;

protected:
  void clear();

  std::string name_;

  /// non-const getLink()
  void getLink(const std::string& name, boost::shared_ptr<Link> &link) const;

  /// non-const getMaterial()
  boost::shared_ptr<Material> getMaterial(const std::string& name) const;
};

}

#endif
