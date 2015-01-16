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
								{};
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

			public: void Parse(const std::string &_filename, std::vector<SVGPath> &paths);
			public: void DumpPaths(const std::vector<SVGPath> paths, std::ostream &out=std::cout) const;

			private: void make_commands(char cmd, const std::vector<double> &numbers, std::vector<SVGCommand> &cmds);
			private: void get_path_commands(const std::vector<std::string> &tokens, SVGPath &path);
			private: void GetPathAttribs(TiXmlElement* pElement, SVGPath &path);
			private: void GetSvgPaths(TiXmlNode* pParent, std::vector<SVGPath> &paths);

			private: void ExpandCommands(const std::vector< std::vector<SVGCommand> > &subpaths, SVGPath &path);
			private: void SplitSubpaths(const std::vector<SVGCommand> cmds, std::vector< std::vector<SVGCommand> > &split_cmds);
			private: void PathToPoints(const SVGPath &path, double resolution, std::vector< std::vector<math::Vector2d> > &polys);

			private: math::Vector2d SubpathToPolyline(const std::vector<SVGCommand> &subpath, math::Vector2d last,
					std::vector<math::Vector2d> &polyline);

            private: double resolution;
		};
	}
}


