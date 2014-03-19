/*
 * Copyright 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _GZ_LOG_HH_
#define _GZ_LOG_HH_

#include <string>
#include <list>

#include <gazebo/physics/WorldState.hh>
#include "gz.hh"

namespace gazebo
{
  /// \brief Base class for all filters.
  class FilterBase
  {
    /// \brief Constructor
    /// \param[in] _xmlOutput True if the output should be in XML format.
    /// \param[in[ _stamp Type of stamp to apply.
    public: FilterBase(bool _xmlOutput, const std::string &_stamp);

    /// \brief Output a line of data.
    /// \param[in] _stream The output stream.
    /// \param[in] _state Current state.
    /// \return Reference to the output stream.
    public: std::ostringstream &Out(std::ostringstream &_stream,
                const gazebo::physics::State &_state);

    /// \brief Filter a pose.
    /// \param[in] _pose The pose to filter.
    /// \param[in] _xmlName Name of the xml tag.
    /// \param[in] _filter The filter string [x,y,z,r,p,a].
    /// \param[in] _state Current state.
    /// \return Filtered pose string.
    public: std::string FilterPose(const gazebo::math::Pose &_pose,
                const std::string &_xmlName,
                std::string _filter,
                const gazebo::physics::State &_state);

    /// \brief True if XML output is requested.
    protected: bool xmlOutput;

    /// \brief Time stamp type
    protected: std::string stamp;
  };

  /// \brief Filter for joint state.
  class JointFilter : public FilterBase
  {
    /// \brief Constructor.
    /// \param[in] _xmlOutput True if the output should be in XML format.
    /// \param[in] _stamp Type of stamp to apply.
    public: JointFilter(bool _xmlOutput, const std::string &_stamp);

    /// \brief Initialize the filter.
    /// \param[in] _filter The command line filter string.
    public: void Init(const std::string &_filter);

    /// \brief Filter joint parts (angle)
    /// \param[in] _state Link state to filter.
    /// \param[in] _partIter Iterator to the filtered string parts.
    /// \return Filtered joint string.
    public: std::string FilterParts(gazebo::physics::JointState &_state,
                std::list<std::string>::iterator _partIter);

    /// \brief Filter the joints in a Model state, and output the result
    /// as a string.
    /// \param[in] _state The model state to filter.
    /// \return Filtered string.
    public: std::string Filter(gazebo::physics::ModelState &_state);

    /// \brief The list of filter strings.
    public: std::list<std::string> parts;
  };

  /// \brief Filter for link state.
  class LinkFilter : public FilterBase
  {
    /// \brief Constructor.
    /// \param[in] _xmlOutput True if the output should be in XML format.
    /// \param[in] _stamp Type of stamp to apply.
    public: LinkFilter(bool _xmlOutput, const std::string &_stamp);

    /// \brief Initialize the filter.
    /// \param[in] _filter The command line filter string.
    public: void Init(const std::string &_filter);

    /// \brief Filter link parts (pose, velocity, acceleration, wrench)
    /// \param[in] _state Link state to filter.
    /// \param[in] _partIter Iterator to the filtered string parts.
    /// \return Filtered string
    public: std::string FilterParts(gazebo::physics::LinkState &_state,
                std::list<std::string>::iterator _partIter);

    /// \brief Filter the links in a Model state, and output the result
    /// as a string.
    /// \param[in] _state The model state to filter.
    /// \return Filtered string.
    public: std::string Filter(gazebo::physics::ModelState &_state);

    /// \brief The list of filter strings.
    public: std::list<std::string> parts;
  };

  /// \brief Filter for model state.
  class ModelFilter : public FilterBase
  {
    /// \brief Constructor.
    /// \param[in] _xmlOutput True if the output should be in XML format.
    /// \param[in] _stamp Type of stamp to apply.
    public: ModelFilter(bool _xmlOutput, const std::string &_stamp);

    /// \brief Destructor.
    public: virtual ~ModelFilter();

    /// \brief Initialize the filter.
    /// \param[in] _filter The command line filter string.
    public: void Init(const std::string &_filter);

    /// \brief Filter model parts (pose)
    /// \param[in] _state Model state to filter.
    /// \param[in] _partIter Iterator to the filtered string parts.
    /// \return Filtered string
    public: std::string FilterParts(gazebo::physics::ModelState &_state,
                std::list<std::string>::iterator _partIter);

    /// \brief Filter the models in a World state, and output the result
    /// as a string.
    /// \param[in] _state The World state to filter.
    /// \return Filtered string.
    public: std::string Filter(gazebo::physics::WorldState &_state);

    /// \brief The list of model parts to filter.
    public: std::list<std::string> parts;

    /// \brief Pointer to the link filter.
    public: LinkFilter *linkFilter;

    /// \brief Pointer to the joint filter.
    public: JointFilter *jointFilter;
  };

  /// \brief Filter interface for an entire state.
  class StateFilter : public FilterBase
  {
    /// \brief Constructor
    /// \param[in] _xmlOutput True to format output as XML
    /// \param[in] _stamp Type of stamp to apply.
    public: StateFilter(bool _xmlOutput, const std::string &_stamp,
                double _hz = 0);

    /// \brief Initialize the filter with a set of parameters.
    /// \param[_in] _filter The filter parameters
    public: void Init(const std::string &_filter);

    /// \brief Perform filtering
    /// \param[in] _stateString The string to filter.
    /// \return Filtered string
    public: std::string Filter(const std::string &_stateString);

    /// \brief Filter for a model.
    private: ModelFilter filter;

    /// \brief Rate at which to output states.
    private: double hz;

    /// \brief Previous time a state was output.
    private: gazebo::common::Time prevTime;
  };

  /// \brief Log command
  class LogCommand : public Command
  {
    /// \brief Constructor
    public: LogCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();

    // Documentation inherited
    protected: virtual bool TransportRequired();

    /// \brief Output information about a log file.
    /// \param[in] _filename Name of the file to parse.
    private: void Info(const std::string &_filename);

    /// \brief Dump the contents of a log file to screen
    /// \param[in] _filter Filter string
    /// \param[in] _raw True to output data without xml formatting.
    /// \param[in] _stamp Type of stamp to apply.
    /// \param[in] _hz Hertz rate.
    private: void Echo(const std::string &_filter,
                 bool _raw, const std::string &_stamp, double _hz);

    /// \brief Step through a log file.
    /// \param[in] _filter Filter string
    /// \param[in] _raw True to output data without xml formatting.
    /// \param[in] _stamp Type of stamp to apply.
    /// \param[in] _hz Hertz rate.
    private: void Step(const std::string &_filter, bool _raw,
                 const std::string &_stamp, double _hz);

    /// \brief Start or stop logging
    /// \param[in] _start True to start logging
    private: void Record(bool _start);

    /// \brief Get a character from the terminal.
    /// This bypasses the need to wait for the 'enter' key.
    /// \return The characater read from the terminal.
    private: int GetChar();

    /// \brief Get the size of file.
    /// \param[in] _filename Name of the file to get the size of.
    /// \return The size of the file in human readable format.
    private: std::string GetFileSizeStr(const std::string &_filename);

    /// \brief Load a log file from a filename.
    /// \param[in] _filename Filename to open
    /// \return True on success.
    private: bool LoadLogFromFile(const std::string &_filename);

    /// \brief Node pointer.
    private: gazebo::transport::NodePtr node;
  };
}
#endif
