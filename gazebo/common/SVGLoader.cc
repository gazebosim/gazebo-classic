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

#include "SVGLoaderPrivate.hh"
#include "SVGLoader.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
std::string lowercase(const std::string &_in)
{
  std::string out = _in;
  std::transform(out.begin(), out.end(), out.begin(), ::tolower);
  return out;
}

/////////////////////////////////////////////////
std::string lowercase(const char *_in)
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
void cubicBezier(const math::Vector2d &_p0,
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
}

/////////////////////////////////////////////////
SvgError::SvgError(const std::string &_what)
  : std::runtime_error(_what)
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
    if (cmd.cmd == 'm' || cmd.cmd == 'l')
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
    else if (cmd.cmd == 'M' || cmd.cmd == 'L')
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
    else if (cmd.cmd == 'C')
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
        cubicBezier(p0, p1, p2, p3, this->dataPtr->resolution, _polyline);
        _last = p3;
        i += 6;
      }
    }
    else if (cmd.cmd == 'c')
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
        cubicBezier(p0, p1, p2, p3, this->dataPtr->resolution, _polyline);
        _last = p3;
        i += 6;
      }
    }
  }
  return _last;
}

/////////////////////////////////////////////////
SVGLoader::SVGLoader(unsigned int _samples)
{
  this->dataPtr = new SVGLoaderPrivate();
  if (_samples == 0)
  {
    std::string m("The number of samples cannot be 0");
    SvgError e(m);
    throw e;
  }
  this->dataPtr->resolution = 1.0/_samples;
}

/////////////////////////////////////////////////
SVGLoader::~SVGLoader()
{
  delete(this->dataPtr);
}

/////////////////////////////////////////////////
void SVGLoader::SplitSubpaths(const std::vector<SVGCommand> &_cmds,
                              std::vector< std::vector<SVGCommand> > &_subpaths)
{
  if (_cmds.empty())
  {
    std::ostringstream os;
    os << "SVGPath has no commands";
    SvgError x(os.str());
    throw x;
  }

  for (SVGCommand cmd: _cmds)
  {
    if (tolower(cmd.cmd) == 'm')
    {
      // the path contains a subpath
      std::vector<SVGCommand> sub;
      _subpaths.push_back(sub);
    }
    // get a reference to the latest subpath
    std::vector<SVGCommand> &subpath = _subpaths.back();
    // give the cmd to the latest
    subpath.push_back(cmd);
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
      if (tolower(xCmd.cmd) == 'c')
        numberCount = 6;
      if (tolower(xCmd.cmd) == 'm')
        numberCount = 2;
      if (tolower(xCmd.cmd) == 'l')
        numberCount = 2;
      if (tolower(xCmd.cmd) == 'v')
        numberCount = 1;
      if (tolower(xCmd.cmd) == 'h')
        numberCount = 1;
      if (tolower(xCmd.cmd) == 'z')
        subpath.push_back(xCmd);
      // group numbers together and repeat the command
      // for each group
      unsigned int n = 0;
      size_t size = xCmd.numbers.size();
      while (n < size)
      {
        subpath.push_back(SVGCommand());
        SVGCommand &cmd = subpath.back();
        cmd.cmd = xCmd.cmd;
        for (size_t i = 0; i < numberCount; ++i)
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
        c.cmd = lastCmd;
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
    c.cmd = lastCmd;
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
void SVGLoader::GetPathAttribs(TiXmlElement *_pElement, SVGPath &_path)
{
  if ( !_pElement ) return;

  TiXmlAttribute *pAttrib = _pElement->FirstAttribute();
  while (pAttrib)
  {
    std::string name = lowercase(pAttrib->Name());
    std::string value = pAttrib->Value();
    if (name == "style")
    {
      _path.style = value;
    }
    else if (name == "id")
    {
      _path.id = value;
    }
    else if (name == "transform")
    {
      _path.transform = value;
    }
    else if (name == "d")
    {
      // this attribute contains a list of coordinates
      std::vector<std::string> tokens;
      split(value, ' ', tokens);
      this->GetPathCommands(tokens, _path);
    }
    pAttrib = pAttrib->Next();
  }
}

/////////////////////////////////////////////////
void SVGLoader::GetSvgPaths(TiXmlNode *_pParent, std::vector<SVGPath> &_paths)
{
  if (!_pParent)
    return;

  TiXmlNode *pChild;
  int t = _pParent->Type();
  std::string name;
  switch ( t )
  {
    case TiXmlNode::TINYXML_ELEMENT:
      name = lowercase(_pParent->Value());
      if (name == "path")
      {
        SVGPath p;
        this->GetPathAttribs(_pParent->ToElement(), p);
        _paths.push_back(p);
      }
      break;

    default:
      break;
  }

  for (pChild = _pParent->FirstChild();
       pChild != 0;
       pChild = pChild->NextSibling())
  {
    this->GetSvgPaths(pChild, _paths);
  }
}

/////////////////////////////////////////////////
bool SVGLoader::Parse(const std::string &_filename,
    std::vector<SVGPath> &_paths)
{
  try
  {
    // load the named file and dump its structure to STDOUT
    TiXmlDocument doc(_filename.c_str());
    bool loadOkay = doc.LoadFile();
    if (!loadOkay)
    {
      std::ostringstream os;
      gzerr << "Failed to load file " <<  _filename << std::endl;
      gzerr << os.str() << std::endl;
    }

    this->GetSvgPaths(&doc, _paths);
    return true;
  }
  catch(SvgError &e)
  {
    gzerr << e.what() << std::endl;
  }
  return false;
}

/////////////////////////////////////////////////
void SVGLoader::DumpPaths(const std::vector<SVGPath> &_paths,
                          std::ostream &_out) const
{
  // this prints an html document that allows to debug
  // SVG parsing issues. The points are generated in
  // a loop between the header and footer.
  std::string header = R"***(
<!DOCTYPE html>
<html>

<script type="text/javascript">

)***";
  std::string footer = R"***(
</script>

<script>

var x0 = 0;
var y0 = 0;
var scale = 1.;

function xx(x)
{
  var r = x0 + scale * x;
  return r;
}

function yy(y)
{
  var r =  - (y0 + scale * (-y) );
  return r;
}

function drawPoint(ctx, x, y)
{
  ctx.beginPath();
  ctx.arc(x, y, 5, 0, 2 * Math.PI, true);
  ctx.strokeStyle= style;
  ctx.stroke();
}

function drawPath(ctx, path, style, x0, y0, scale, showCtrlPoints )
{
  console.log('drawPath ' + path.name);

  ctx.beginPath();
  for (var j = 0; j <  path.subpaths.length; j++)
  {
    var points = path.subpaths[j];
    console.log(points.length + ' points in subpath, (' + style + ')');
    if (points.length < 2)
    {
      console.log("not enough points in subpath " + j);
      return;
    }
    ctx.moveTo(xx(points[0][0]), yy(points[0][1]));
    for (var i = 1; i < points.length; ++i)
    {
      var x= xx(points[i][0]);
      var y= yy(points[i][1]);
      ctx.lineTo(x, y);
    }
    ctx.strokeStyle= style;
    ctx.stroke();

    // draw points
    if (showCtrlPoints)
    {
      var styles = ["black", "orange", "grey"];
      for (var i = 0; i < points.length; ++i)
      {
        var x= xx(points[i][0]);
        var y= yy(points[i][1]);
        var m = " [" + points[i][0] + ", " + points[i][1];
        m += "]  [" + x + ", " + y + "]";
        console.log(m);
        ctx.beginPath();
        if (i == 0)
        {
          ctx.arc(x, y, 4, 0, 2 * Math.PI, true);
          ctx.strokeStyle = "red";
          ctx.fill();
        }
        else if (i == 1)
        {
          ctx.arc(x, y, 2, 0, 2 * Math.PI, true);
          ctx.strokeStyle= "red";
        }
        else
        {
          ctx.arc(x, y, 2, 0, 2 * Math.PI, true);
          ctx.strokeStyle= styles[i % styles.length ];
        }
        ctx.stroke();
       }
    }
  }
}


function draw(showCtrlPoints)
{
  var canvas = document.getElementById("myCanvas");
  var ctx = canvas.getContext("2d");
  var styles = ["red", "green", "blue"];

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  x0 = Number(document.getElementsByName("xoff_in")[0].value);
  y0 = Number(document.getElementsByName("yoff_in")[0].value);
  scale = Number(document.getElementsByName("scale_in")[0].value);

  for (var i =0; i < svg.length; ++i)
  {
    var path = svg[i];
    console.log("path: " + path.name);
    drawPath(ctx, path, styles[i%3], x0, y0, scale, showCtrlPoints);
  }
}

  console.log("number of paths: " + svg.length);

  document.addEventListener("DOMContentLoaded", function(event)
  {
    draw();
  });

</script>


<body>

  <div>

  Xoff: <input type="text" name="xoff_in" value="0"><br>
  Yoff: <input type="text" name="yoff_in" value="0"><br>
  Scale: <input type="text" name="scale_in" value="1.0"><br>

  <button onclick="draw(true);">Draw</button>
  </div>

  <canvas
    id="myCanvas"
    width="1024"
    height="768"
    style="border:1px solid #d3d3d3;">
    Your browser does not support the canvas element.
  </canvas>

</body>
</html>

)***";

  _out << header << std::endl;
  _out << "var svg = [];" << std::endl;
  for (SVGPath path : _paths)
  {
    _out << "svg.push({name:\"" << path.id;
    _out <<  "\", subpaths:[], style: \"";
    _out << path.style << "\"}); " << std::endl;
    _out << "svg[svg.length-1].subpaths = [";
    char psep = ' ';

    for (unsigned int i = 0; i < path.polylines.size(); ++i)
    {
      std::vector<math::Vector2d> poly = path.polylines[i];
      _out << psep <<  "[" << std::endl;
      psep = ',';
      char sep = ' ';
      for ( math::Vector2d p : poly)
      {
        _out << " " << sep << " [" <<  p.x << ", " << p.y << "]" <<std::endl;
        sep = ',';
      }
      _out << " ] " << std::endl;
    }
    _out << "];" << std::endl;
    _out << "\n\n";
  }
  _out << footer << std::endl;
}
