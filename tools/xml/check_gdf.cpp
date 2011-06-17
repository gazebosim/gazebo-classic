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

#include "parser.h"
#include <iostream>

using namespace gdf;

int main(int argc, char** argv)
{
  if (argc < 2){
    std::cerr << "Expect xml file to parse" << std::endl;
    return -1;
  }

  TiXmlDocument model_doc;
  model_doc.LoadFile(argv[1]);

  TiXmlElement *model_xml = model_doc.FirstChildElement("model");

  if (!model_xml){
    std::cerr << "ERROR: Could not load the xml into TiXmlElement" << std::endl;
    return -1;
  }

  Parser model;
  if (!model.init(model_xml)){
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return -1;
  }

  std::cout << "robot name is: " << model.getName() << std::endl;

  // get info from parser
  std::cout << "---------- Successfully Parsed XML ---------------" << std::endl;
  // get root link
  std::vector<boost::shared_ptr<Link> > links;
  model.getLinks(links);

  std::map<std::string, boost::shared_ptr<Joint> >::iterator jIter;
  
  for (std::vector<boost::shared_ptr<Link> >::iterator iter = links.begin(); iter != links.end(); iter++)
  {
    std::cout << "link: " << (*iter)->name << "\n";
    std::vector<boost::shared_ptr<Visual> >::iterator vIter;
    std::vector<boost::shared_ptr<Collision> >::iterator cIter;

    std::map<std::string, boost::shared_ptr<Sensor> >::iterator sIter;

    for (vIter = (*iter)->visuals.begin();
         vIter != (*iter)->visuals.end(); vIter++)
    {
      std::cout << "  Visual: " << (*vIter)->name << "\n";
    }

    for (cIter = (*iter)->collisions.begin();
         cIter != (*iter)->collisions.end(); cIter++)
    {
      std::cout << "  Collision: " << (*cIter)->name << "\n";
    }

    for (sIter = (*iter)->sensors.begin(); sIter != (*iter)->sensors.end(); sIter++)
    {
      std::cout << "sensor: " << sIter->second->name << "\n";
    }


  }

  for (jIter = model.joints_.begin(); jIter != model.joints_.end(); jIter++)
  {
    std::cout << "joint: " << jIter->second->name << "\n";
  }

  return 0;
}

