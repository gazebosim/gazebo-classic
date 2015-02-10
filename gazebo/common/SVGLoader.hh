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

#ifndef SVGLOADER_HH
#define SVGLOADER_HH

#include <stdexcept>
#include <string>
#include <vector>

#include <gazebo/math/Vector2d.hh>

class TiXmlElement;
class TiXmlNode;

namespace gazebo
{
  namespace common
  {
    class GAZEBO_VISIBLE SvgError: public std::runtime_error
    {
      public: SvgError(const std::string& what_arg)
        : std::runtime_error(what_arg)
      {}
    };

    struct GAZEBO_VISIBLE SVGCommand
    {
      char type;
      std::vector<double> numbers;
    };

    struct GAZEBO_VISIBLE SVGPath
    {
      std::string id;
      std::string style;
      std::string transform;
      std::vector< std::vector<SVGCommand> > subpaths;
      std::vector< std::vector<math::Vector2d> > polylines;
    };

    class GAZEBO_VISIBLE SVGLoader
    {
      public: SVGLoader(unsigned int _samples);
      public: void Parse(const std::string &_filename,
                         std::vector<SVGPath> &_paths);
      public: void DumpPaths(const std::vector<SVGPath> &_paths, std::ostream &out) const;

      private: void MakeCommands( char _cmd,
                                  const std::vector<double> &_numbers,
                                  std::vector<SVGCommand> &_cmds);
      private: void GetPathCommands(const std::vector<std::string> &_tokens, SVGPath &_path);
      private: void GetPathAttribs(TiXmlElement* _pElement, SVGPath &_path);
      private: void GetSvgPaths(TiXmlNode* _pParent,
                                  std::vector<SVGPath> &_paths);
      private: void ExpandCommands(
                      const std::vector< std::vector<SVGCommand> >&_subpaths,
                      SVGPath &_path);
      private: void SplitSubpaths(const std::vector<SVGCommand> &_cmds,
                      std::vector< std::vector<SVGCommand> > &_split_cmds);
      private: void PathToPoints(const SVGPath &_path,
                      double _resolution,
                      std::vector< std::vector<math::Vector2d> > &_polys);
      private: math::Vector2d SubpathToPolyline(
                      const std::vector<SVGCommand> &_subpath,
                      math::Vector2d _last,
                      std::vector<math::Vector2d> &_polyline);
      private: double resolution;
		};
	}
}

#endif

