/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef GAZEBO_PLUGINS_SHADERPARAMVISUALPLUGIN_HH_
#define GAZEBO_PLUGINS_SHADERPARAMVISUALPLUGIN_HH_

#include <memory>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/RenderTypes.hh>

namespace gazebo
{
  // Forward declare private data class.
  class ShaderParamVisualPluginPrivate;

  /// \brief A plugin that demonstrates how to set shader parameters
  /// of a material used by a visual
  ///
  /// Plugin parameters:
  ///
  /// <param>    Shader parameter - can be repeated within plugin SDF element
  ///   <name>   Name of uniform bound to the shader
  ///   <type>   Type of shader, i.e. vertex, fragment
  ///   <value>  Value to set the shader parameter to. The value string can be
  ///            an int, float, or a space delimited array of floats.
  ///            It can also be 'TIME', in which case the value will be bound
  ///            to sim time.
  ///
  /// Example usage:
  ///
  /// \verbatim
  ///    <plugin name="shader_param" filename="libShaderParamVisualPlugin.so">
  ///
  ///      <!-- Sets a fragment shader uniform name "ambient" to color red -->
  ///      <param>
  ///        <name>ambient</name>
  ///        <type>fragment</name>
  ///        <value>1.0 0.0 0.0 1.0</value>
  ///      </param>
  ///    </plugin>
  /// \endverbatim
  class GAZEBO_VISIBLE ShaderParamVisualPlugin : public VisualPlugin
  {
    /// \brief Constructor.
    public: ShaderParamVisualPlugin();

    /// \brief Destructor.
    public: ~ShaderParamVisualPlugin();

    // Documentation inherited
    public: virtual void Load(rendering::VisualPtr _visual,
        sdf::ElementPtr _sdf);

    /// \brief Update the plugin once every iteration of simulation.
    private: void Update();

    /// \brief Callback to receive pose info.
    private: void OnInfo(ConstPosesStampedPtr &_msg);

    /// \internal
    /// \brief Private data pointer
    private: std::unique_ptr<ShaderParamVisualPluginPrivate> dataPtr;
  };
}
#endif
