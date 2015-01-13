/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "SVGLoader.hh"
#include <tinyxml.h>
#include <algorithm>

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
std::string lowercase(const std::string& in)
{
  std::string out = in;
  std::transform(out.begin(), out.end(), out.begin(), ::tolower);
  return out;
}

/////////////////////////////////////////////////
std::string lowercase(const char* in)
{
  std::string ins = in;
  return lowercase(ins);
}

/////////////////////////////////////////////////
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

/////////////////////////////////////////////////
math::Vector2d bezierInterpolate(double t, const math::Vector2d &p0, const math::Vector2d &p1, const math::Vector2d &p2, const math::Vector2d &p3)
{
  double t_1 = 1.0 - t;
  double t_1_2 = t_1 * t_1;
  double t_1_3 = t_1_2 * t_1;
  double t2 = t * t;
  double t3 = t2 * t;
  
  math::Vector2d p;  
  p.x = t_1_3 * p0.x + 3 * t *  t_1_2 * p1.x + 3 * t2 * t_1 * p2.x + t3 * p3.x;
  p.y = t_1_3 * p0.y + 3 * t *  t_1_2 * p1.y + 3 * t2 * t_1 * p2.y + t3 * p3.y;

  return p;  
}

/////////////////////////////////////////////////
double Distance(const math::Vector2d &p0, const math::Vector2d &p1)
{
  double xx = (p0.x - p1.x) *  (p0.x - p1.x);
  double yy = (p0.y - p1.y) *  (p0.y - p1.y);
  return sqrt(xx + yy);
}

/////////////////////////////////////////////////
unsigned int GetStepCount(const math::Vector2d &p0, const math::Vector2d &p1, const math::Vector2d &p2, const math::Vector2d &p3, double res)
{
  double d = Distance(p0,p1) + Distance(p1,p2) + Distance(p2, p3);
  double steps = res / d;
  return (unsigned int) fabs(steps);
}

/////////////////////////////////////////////////
math::Vector2d SVGLoader::SubpathToPolyline(const std::vector<SVGCommand> &subpath, math::Vector2d last,  std::vector<math::Vector2d> &polyline)
{
  for (SVGCommand cmd: subpath)
  {
    if (cmd.type == 'm' || cmd.type == 'l')
    {
       size_t i =0;
       size_t count = cmd.numbers.size();
       while(i < count)
       {
         math::Vector2d p;
         p.x = cmd.numbers[i+0];
         p.y = cmd.numbers[i+1];
         // m and l cmds are relative to the last point
         p.x += last.x;
         p.y += last.y;
         polyline.push_back(p);
         last = p; 
         i += 2;
      }
    }
  }
  return last;
}

/////////////////////////////////////////////////
void SVGLoader::SplitSubpaths(const std::vector<SVGCommand> cmds, std::vector< std::vector<SVGCommand> > &subpaths)
{
  if(cmds.size() ==0)
  {
    std::ostringstream os;
    os << "SVGPath has no commands";
    SvgError x(os.str());
    throw x;
  }
  
  for(SVGCommand cmd: cmds)
  {
    if( tolower(cmd.type) == 'm')
    {
      // the path contains a subpath
      std::vector<SVGCommand> sub;
      subpaths.push_back(sub);
    }
    // get a reference to the latest subpath
    std::vector<SVGCommand> &subpath = subpaths.back();
    // give the cmd to the latest
    subpath.push_back(cmd);
  }  
}

/////////////////////////////////////////////////
void SVGLoader::make_commands(char cmd, const std::vector<double> &numbers, std::vector<SVGCommand> &cmds)
{
  if(cmd != 'x')
  { 
    SVGCommand c;
    c.type = cmd;
    c.numbers = numbers;
    cmds.push_back(c);
  }


}

/////////////////////////////////////////////////
void SVGLoader::ExpandCommands(const std::vector< std::vector<SVGCommand> > &subpaths,  SVGPath &path)
{
  for (std::vector<SVGCommand> compressedSubpath :subpaths)
  {
    // add new subpath
    path.subpaths.push_back( std::vector<SVGCommand>());
    // get a reference	
    std::vector<SVGCommand> &subpath = path.subpaths.back();
    // copy the cmds with repeating commands, grouping the numbers
    for (SVGCommand xCmd : compressedSubpath)
    {
      unsigned int numberCount = 0;      
      if (tolower(xCmd.type) == 'c') 
	numberCount = 6;
      if (tolower(xCmd.type) == 'm')
	numberCount = 2;
      if (tolower(xCmd.type) == 'l')
	numberCount = 2;
      if (tolower(xCmd.type) == 'v')
	numberCount = 1;
      if (tolower(xCmd.type) == 'h')
	numberCount = 1;
      if (tolower(xCmd.type) == 'z')
      {
        subpath.push_back(xCmd);
      }	
      
      // group numbers together and repeat the command
      // for each group
      unsigned int n = 0;
      size_t size = xCmd.numbers.size();

      while(n < size)
      {
        subpath.push_back(SVGCommand());
        SVGCommand &cmd = subpath.back();
        cmd.type = xCmd.type;
	
     	for(size_t i=0; i < numberCount; i++)
	    {
	      cmd.numbers.push_back(xCmd.numbers[i+n]);
	    }
        n += numberCount;
      } 
    }
  }
}

/////////////////////////////////////////////////
void SVGLoader::get_path_commands(const std::vector<std::string> &tokens, SVGPath &path)
{
     std::vector <SVGCommand> cmds;
     std::string lookup = "cCmMlLvVhHzZ";
     char lastCmd = 'x';
     std::vector<double> numbers;
     
     for(std::string token: tokens)
     {
       // new command?
       if(lookup.find(token[0]) == std::string::npos)
       { 
         // its just numbers
         std::cout << std::endl;
         std::vector<std::string> numberStrs;
         split(token, ',', numberStrs);
         for(std::string numberStr : numberStrs)
         {
           double f = atof(numberStr.c_str());
           numbers.push_back(f);
         }
       }
       else
       {
         
         if(lastCmd != 'x')
         { 
           SVGCommand c;
           c.type = lastCmd;
           c.numbers = numbers;
           cmds.push_back(c);
         }

         // its new command
         lastCmd = token[0];
         numbers.resize(0);
       }
     }
     // the last command
     if(lastCmd != 'x')
     { 
       SVGCommand c;
       c.type = lastCmd;
       c.numbers = numbers;
       cmds.push_back(c);
     }
    // split the commands into sub_paths 
    std::vector< std::vector< SVGCommand> > subpaths;
    this->SplitSubpaths(cmds, subpaths);

    this->ExpandCommands(subpaths, path );

    // the starting point for the subpath
    // it is the end point of the previous one
    math::Vector2d p;
    p.x = 0;
    p.y = 0;
    for (std::vector<SVGCommand> subpath : subpaths)
    {
        path.polylines.push_back(std::vector<math::Vector2d>());
        std::vector<math::Vector2d> &polyline = path.polylines.back();
        
    	std::cout << "XXX " << polyline.size() << std::endl;
        p = this->SubpathToPolyline(subpath, p, polyline);
        std::cout << "X X X " << polyline.size() << std::endl;
    }
        
}

/////////////////////////////////////////////////
void SVGLoader::get_path_attribs(TiXmlElement* pElement, SVGPath &path)
{
    if ( !pElement ) return;

    TiXmlAttribute* pAttrib=pElement->FirstAttribute();
    while (pAttrib)
    {
        std::string name = lowercase(pAttrib->Name());
        std::string value= lowercase(pAttrib->Value());
        if (name == "style")
        {
            path.style = value;
        }
        if (name == "id")
        {
            path.id = value;
        }
        if (name == "d")
        {
            using namespace std;
            // this attribute contains a list of coordinates
            std::vector<std::string> tokens;
            split(value, ' ', tokens);
            get_path_commands(tokens, path);
            
        }
        // int ival;
        // double dval;
        // if (pAttrib->QueryIntValue(&ival)==TIXML_SUCCESS)    printf( " int=%d", ival);
        // if (pAttrib->QueryDoubleValue(&dval)==TIXML_SUCCESS) printf( " d=%1.1f", dval);
        pAttrib=pAttrib->Next();
    }
}

/////////////////////////////////////////////////
void SVGLoader::get_svg_paths(TiXmlNode* pParent, std::vector<SVGPath> &paths)
{
  if (!pParent) 
		return;

  TiXmlNode *pChild;
  int t = pParent->Type();
  std::string name;
  switch ( t )
  {
    case TiXmlNode::TINYXML_ELEMENT:
      name = lowercase(pParent->Value());
      if (name == "path")
      {
        SVGPath p;
        get_path_attribs(pParent->ToElement(), p);
        paths.push_back(p);
      }
      break; 

		default:
    	break;
   }

  for (pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
  {
		get_svg_paths( pChild, paths );
  }
}

/////////////////////////////////////////////////
void SVGLoader::Parse(const std::string &_filename, std::vector<SVGPath> &paths)
{
    // load the named file and dump its structure to STDOUT
    TiXmlDocument doc(_filename.c_str());
    bool loadOkay = doc.LoadFile();
    if (!loadOkay)
    {
      std::ostringstream os;
      os << "Failed to load file " <<  _filename;
      SvgError x(os.str());
      throw x;
    }

  get_svg_paths( &doc, paths);

}

/////////////////////////////////////////////////
void SVGLoader::Dump_paths(const std::vector<SVGPath> paths ) const
{
  std::cout << "var svg = [];" << std::endl;
  for (SVGPath path : paths)
  {
    std::cout << "svg.push({name:\"" << path.id <<  "\", subpaths:[], style: \"" << path.style << "\"}); " << std::endl;
    // std::cout << " -" << path.id << " " << path.style << std::endl;
//    for (std::vector<SVGCommand> subpath : path.subpaths)
//    {
      // std::cout << "//  subpath (" << subpath.size() << " cmds)" << std::endl;
      // for (SVGCommand cmd: subpath)
      // {
        // std::cout << "//    " << cmd.tostr() << std::endl;
     // }
//    }
    std::cout << "svg[svg.length-1].subpaths = [";
    char psep = ' ';
    for (unsigned int i=0; i < path.polylines.size(); i++)
    {
      std::vector<math::Vector2d> poly = path.polylines[i];
      std::cout << psep <<  "[" << std::endl;
      psep = ',';
      char sep = ' ';
      for( math::Vector2d p : poly)
      {
        std::cout << " " << sep << " [" <<  p.x << ", " << p.y << "]" <<std::endl;
	    sep = ',';
      }
      std::cout << " ] " << std::endl;
    }
    std::cout << "];" << std::endl;
    std::cout << "\n\n";
  }
}

// ----------------------------------------------------------------------
// main() for printing files named on the command line
// ----------------------------------------------------------------------
/*int main(int argc, char* argv[])
{
    std::vector<SVGPath> paths;

    for (int i=1; i<argc; i++)
    {
      std::cout << "=========\nFILE: " << argv[i] << std::endl;
      std::vector<SVGPath> paths;

      SVGLoader svg;
      svg.Parse(argv[i], paths);
      svg.Dump_paths(paths);     
    }

    return 0;
}*/

