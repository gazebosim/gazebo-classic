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

#include <algorithm>
#include <tinyxml.h>
#include "SVGLoader.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
std::string lowercase(const std::string& _in)
{
  std::string out = _in;
  std::transform(out.begin(), out.end(), out.begin(), ::tolower);
  return out;
}

/////////////////////////////////////////////////
std::string lowercase(const char* _in)
{
  std::string ins = _in;
  return lowercase(ins);
}

/////////////////////////////////////////////////
std::vector<std::string> &split(const std::string &_s,
                                char _delim,
                                std::vector<std::string> &_elems)
{
    std::stringstream ss(_s);
    std::string item;
    while (std::getline(ss, item, _delim))
    {
        _elems.push_back(item);
    }
    return _elems;
}

/////////////////////////////////////////////////
math::Vector2d bezierInterpolate(double _t,
                                 const math::Vector2d &_p0,
                                 const math::Vector2d &_p1,
                                 const math::Vector2d &_p2,
                                 const math::Vector2d &_p3)
{
  double t_1 = 1.0 - _t;
  double t_1_2 = t_1 * t_1;
  double t_1_3 = t_1_2 * t_1;
  double t2 = _t * _t;
  double t3 = t2 * _t;
  math::Vector2d p;
  p.x = t_1_3 * _p0.x + 3 * _t *  t_1_2 * _p1.x + 3 * t2 * t_1 * _p2.x +
        t3 * _p3.x;
  p.y = t_1_3 * _p0.y + 3 * _t *  t_1_2 * _p1.y + 3 * t2 * t_1 * _p2.y +
        t3 * _p3.y;
  return p;
}

/////////////////////////////////////////////////
void CubicBezier(const math::Vector2d &_p0,
                 const math::Vector2d &_p1,
                 const math::Vector2d &_p2,
                 const math::Vector2d &_p3,
                 double _step,
                 std::vector<math::Vector2d> &_points)
{
  double t = _step;
  while (t < 1.0)
  {
    auto p = bezierInterpolate(t, _p0, _p1, _p2, _p3);
    _points.push_back(p);
    t += _step;
  }
  // auto p = bezierInterpolate(1.0, _p0, _p1, _p2, _p3);
  // _points.push_back(p);
}


/////////////////////////////////////////////////
SvgError::SvgError(const std::string& _what)
  :std::runtime_error(_what)
{
}

/////////////////////////////////////////////////
math::Vector2d SVGLoader::SubpathToPolyline(
                            const std::vector<SVGCommand> &_subpath,
                            math::Vector2d _last,
                            std::vector<math::Vector2d> &_polyline)
{
  for (SVGCommand cmd: _subpath)
  {
    if (cmd.type == 'm' || cmd.type == 'l')
    {
       size_t i =0;
       size_t count = cmd.numbers.size();
       while (i < count)
       {
         math::Vector2d p;
         p.x = cmd.numbers[i+0];
         p.y = cmd.numbers[i+1];
         // m and l cmds are relative to the last point
         p.x += _last.x;
         p.y += _last.y;
         _polyline.push_back(p);
         _last = p;
         i += 2;
      }
    }
    else if (cmd.type == 'M' || cmd.type == 'L')
    {
      size_t i = 0;
      size_t count = cmd.numbers.size();
      while (i < count)
      {
        math::Vector2d p;
        p.x = cmd.numbers[i+0];
        p.y = cmd.numbers[i+1];
        _polyline.push_back(p);
        _last = p;
        i += 2;
      }
    }
    else if (cmd.type == 'C')
    {
      size_t i = 0;
      size_t count = cmd.numbers.size();
      while (i < count)
      {
        math::Vector2d p0 = _last;
        math::Vector2d p1, p2, p3;
        p1.x = cmd.numbers[i+0];
        p1.y = cmd.numbers[i+1];
        p2.x = cmd.numbers[i+2];
        p2.y = cmd.numbers[i+3];
        p3.x = cmd.numbers[i+4];
        p3.y = cmd.numbers[i+5];
        CubicBezier(p0, p1, p2, p3, resolution, _polyline);
        _last = p3;
        i += 6;
      }
    }
    else if (cmd.type == 'c')
    {
      size_t i = 0;
      size_t count = cmd.numbers.size();
      while (i < count)
      {
        math::Vector2d p0 = _last;
        math::Vector2d p1, p2, p3;
        p1.x = cmd.numbers[i+0] + _last.x;
        p1.y = cmd.numbers[i+1] + _last.y;
        p2.x = cmd.numbers[i+2] + _last.x;
        p2.y = cmd.numbers[i+3] + _last.y;
        p3.x = cmd.numbers[i+4] + _last.x;
        p3.y = cmd.numbers[i+5] + _last.y;
        CubicBezier(p0, p1, p2, p3, resolution, _polyline);
        _last = p3;
        i += 6;
      }
    }
  }
  return _last;
}

/////////////////////////////////////////////////
SVGLoader::SVGLoader(unsigned int _samples)
  :resolution(1.0/_samples)
{
}

/////////////////////////////////////////////////
void SVGLoader::SplitSubpaths(const std::vector<SVGCommand> &_cmds,
                              std::vector< std::vector<SVGCommand> > &subpaths)
{
  if (_cmds.size() ==0)
  {
    std::ostringstream os;
    os << "SVGPath has no commands";
    SvgError x(os.str());
    throw x;
  }

  for (SVGCommand cmd: _cmds)
  {
    if (tolower(cmd.type) == 'm')
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
void SVGLoader::MakeCommands(char _cmd,
                             const std::vector<double> &_numbers,
                             std::vector<SVGCommand> &_cmds)
{
  if (_cmd != 'x')
  {
    SVGCommand c;
    c.type = _cmd;
    c.numbers = _numbers;
    _cmds.push_back(c);
  }
}

/////////////////////////////////////////////////
void SVGLoader::ExpandCommands(
                  const std::vector< std::vector<SVGCommand> > &_subpaths,
                  SVGPath &_path)
{
  for (std::vector<SVGCommand> compressedSubpath :_subpaths)
  {
    // add new subpath
    _path.subpaths.push_back(std::vector<SVGCommand>());
    // get a reference
    std::vector<SVGCommand> &subpath = _path.subpaths.back();
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
        subpath.push_back(xCmd);
      // group numbers together and repeat the command
      // for each group
      unsigned int n = 0;
      size_t size = xCmd.numbers.size();
      while (n < size)
      {
        subpath.push_back(SVGCommand());
        SVGCommand &cmd = subpath.back();
        cmd.type = xCmd.type;
        for (size_t i = 0; i < numberCount; i++)
        {
          cmd.numbers.push_back(xCmd.numbers[i+n]);
        }
        n += numberCount;
      }
    }
  }
}

/////////////////////////////////////////////////
void SVGLoader::GetPathCommands(const std::vector<std::string> &_tokens,
                                  SVGPath &_path)
{
     std::vector <SVGCommand> cmds;
     std::string lookup = "cCmMlLvVhHzZ";
     char lastCmd = 'x';
     std::vector<double> numbers;

     for (std::string token: _tokens)
     {
       // new command?
       if (lookup.find(token[0]) == std::string::npos)
       {
         // its just numbers
         std::vector<std::string> numberStrs;
         split(token, ',', numberStrs);
         for (std::string numberStr : numberStrs)
         {
           double f = atof(numberStr.c_str());
           numbers.push_back(f);
         }
       }
       else
       {
         if (lastCmd != 'x')
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
     if (lastCmd != 'x')
     {
       SVGCommand c;
       c.type = lastCmd;
       c.numbers = numbers;
       cmds.push_back(c);
     }
    // split the commands into sub_paths
    std::vector< std::vector< SVGCommand> > subpaths;
    this->SplitSubpaths(cmds, subpaths);
    this->ExpandCommands(subpaths, _path);
    // the starting point for the subpath
    // it is the end point of the previous one
    math::Vector2d p;
    p.x = 0;
    p.y = 0;
    for (std::vector<SVGCommand> subpath : subpaths)
    {
        _path.polylines.push_back(std::vector<math::Vector2d>());
        std::vector<math::Vector2d> &polyline = _path.polylines.back();
        p = this->SubpathToPolyline(subpath, p, polyline);
    }
}

/////////////////////////////////////////////////
void SVGLoader::GetPathAttribs(TiXmlElement* pElement, SVGPath &path)
{
    if ( !pElement ) return;

    TiXmlAttribute* pAttrib = pElement->FirstAttribute();
    while (pAttrib)
    {
        std::string name = lowercase(pAttrib->Name());
        std::string value = pAttrib->Value();
        if (name == "style")
        {
            path.style = value;
        }
        if (name == "id")
        {
            path.id = value;
        }
        if (name == "transform")
        {
            path.transform = value;
        }
        if (name == "d")
        {
            using namespace std;
            // this attribute contains a list of coordinates
            std::vector<std::string> tokens;
            split(value, ' ', tokens);
            GetPathCommands(tokens, path);
        }
        pAttrib = pAttrib->Next();
    }
}

/////////////////////////////////////////////////
void SVGLoader::GetSvgPaths(TiXmlNode* pParent, std::vector<SVGPath> &paths)
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
        GetPathAttribs(pParent->ToElement(), p);
        paths.push_back(p);
      }
      break;

    default:
      break;
  }

  for (pChild = pParent->FirstChild();
        pChild != 0;
        pChild = pChild->NextSibling())
  {
    GetSvgPaths(pChild, paths);
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

  GetSvgPaths(&doc, paths);
}

/////////////////////////////////////////////////
void SVGLoader::DumpPaths(const std::vector<SVGPath> &_paths,
                          std::ostream &out) const
{
  out << "var svg = [];" << std::endl;
  for (SVGPath path : _paths)
  {
    out << "svg.push({name:\"" << path.id;
    out <<  "\", subpaths:[], style: \"";
    out << path.style << "\"}); " << std::endl;
    out << "svg[svg.length-1].subpaths = [";
    char psep = ' ';

    for (unsigned int i = 0; i < path.polylines.size(); i++)
    {
      std::vector<math::Vector2d> poly = path.polylines[i];
      out << psep <<  "[" << std::endl;
      psep = ',';
      char sep = ' ';
      for ( math::Vector2d p : poly)
      {
        out << " " << sep << " [" <<  p.x << ", " << p.y << "]" <<std::endl;
        sep = ',';
      }
      out << " ] " << std::endl;
    }
    out << "];" << std::endl;
    out << "\n\n";
  }
}
