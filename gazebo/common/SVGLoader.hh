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

#ifndef _GAZEBO_SVGLOADER_HH_
#define _GAZEBO_SVGLOADER_HH_

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
    class SVGLoaderPrivate;

    /// \brief Handles errors during SVG parsing
    class GAZEBO_VISIBLE SvgError: public std::runtime_error
    {
      /// \brief constructor
      /// \param[in] _what The error description
      public: SvgError(const std::string &_what);
    };

    /// \brief SVG command data structure
    struct GAZEBO_VISIBLE SVGCommand
    {
      /// \brief A letter that describe the segment
      char cmd;  // cppcheck style error is a false positive

      /// \brief Coordinates for the command
      std::vector<double> numbers;
    };

    /// \brief An SVG path element data structure
    struct GAZEBO_VISIBLE SVGPath
    {
      /// \brief An id or name
      std::string id;

      /// \brief The style (i.e. stroke style, color, thickness etc)
      std::string style;

      /// \brief A 2D transform (or a list of transforms)
      std::string transform;

      /// \brief A list of subpaths (as lists of commands)
      std::vector< std::vector<SVGCommand> > subpaths;

      /// \brief The polylines described by the commands
      std::vector< std::vector<math::Vector2d> > polylines;
    };

    /// \brief A loader for SVG files
    class GAZEBO_VISIBLE SVGLoader
    {
      /// \brief Constructor
      /// \param[in] _samples The number of points for cubic spline segments
      public: SVGLoader(unsigned int _samples);

      /// \brief destructor
      public: ~SVGLoader();

      /// \brief Reads an SVG file and loads all the paths
      /// \param[in] _filename The SVG file
      /// \param[out] _paths Vector that receives path datai
      /// \throws SvgError When the file cannot be processed
      public: void Parse(const std::string &_filename,
                         std::vector<SVGPath> &_paths);

      /// \brief Outputs the content of the paths to file (or console)
      /// \param[in] _paths The paths
      /// \param[in] _out The output stream (can be a file or std::cout)
      public: void DumpPaths(const std::vector<SVGPath> &_paths,
                             std::ostream &_out) const;

      /// \brief Parses a list of strings into a path
      /// \param[in] _tokens The tokenized path attribute from SVG file
      /// \param[out] _path The path that receives the data
      private: void GetPathCommands(const std::vector<std::string> &_tokens,
                                    SVGPath &_path);

      /// \brief Gets data from an XML path element
      /// \param[in] _pElement The path Element
      /// \param[out] _path The path that receives the data.
      private: void GetPathAttribs(TiXmlElement *_pElement, SVGPath &_path);

      /// \brief Reads the paths from the root XML element
      /// \param[in] _pParent The parent XML node of the SVG file
      /// \param[out] _paths The vector of paths that receives the data
      private: void GetSvgPaths(TiXmlNode *_pParent,
                                  std::vector<SVGPath> &_paths);

      /// \brief Generates new commands for every repeat commands in subpaths
      /// \param[in] _subpaths The subpaths (with repeats)
      /// \param[out] _path The path that receives the data.
      private: void ExpandCommands(
                      const std::vector< std::vector<SVGCommand> > &_subpaths,
                      SVGPath& _path);

      /// \brief Splits a list of commands into subpaths
      /// \param[in] _cmds The flat list of commands for all the subpaths
      /// \param[out] _subpaths The vector of subpathts that receives the data
      /// \throws SvgError
      private: void SplitSubpaths(const std::vector<SVGCommand> &_cmds,
                      std::vector< std::vector<SVGCommand> > &_subpaths);

      /// \brief Generates a list of polylines from a path
      /// \param[in] _path The path
      /// \param[in] _resolution The step size (between 0 and 1)
      /// \param[out] _polys The vector that receives the polylines
      private: void PathToPoints(const SVGPath &_path,
                      double _resolution,
                      std::vector< std::vector<math::Vector2d> > &_polys);

      /// \brief Generates polylines for each SVG subpath
      /// \param[in] _subpath The subpath commands
      /// \param[in] _last The previous position (for relative path commands)
      /// \param[out] _polyline The polyline that receives the data
      /// \return The last point of the subpath
      private: math::Vector2d SubpathToPolyline(
                      const std::vector<SVGCommand> &_subpath,
                      math::Vector2d _last,
                      std::vector<math::Vector2d> &_polyline);

      /// \internal
      /// \brief Pointer to private data
      private: SVGLoaderPrivate *dataPtr;
    };
  }
}

#endif
