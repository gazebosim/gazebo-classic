/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#ifndef _GUI_TIMER_PLUGIN_HH_
#define _GUI_TIMER_PLUGIN_HH_

#include <string>
#include <vector>
#include <boost/thread/mutex.hpp>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Timer.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace gazebo
{
  /// \brief A GUI plugin that displays a timer. Control of this timer
  /// is managed via a topic that is ~/timer_control by default. The
  /// topic may be specified in the plugin's SDF. The topic receives
  /// string messages where:
  ///
  ///     "start" == start the timer
  ///     "stop" == stop the timer
  ///     "reset" == reset the timer
  ///
  /// <plugin name="timer_plugin" filename="libTimerGUIPlugin.so">
  ///   <topic>~/my_timer_control</topic>
  ///   <pos>pixel_x_pos pixel_y_pos</pos>
  ///   <size>pixel_width pixel_height</size>
  ///   <start_stop_button>visible</start_stop_button>
  ///   <reset_button>visible</reset_button>
  /// </plugin>
  ///
  /// Note that the size has to be large enough to fit all widgets.
  class GAZEBO_VISIBLE TimerGUIPlugin : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: TimerGUIPlugin();

    /// \brief Destructor
    public: virtual ~TimerGUIPlugin();

    // Documentation inherited
    public: void Load(sdf::ElementPtr _elem);

    /// \brief Get the current time listed in the timer.
    /// \return Gazebo time object representing the time.
    public: common::Time GetCurrentTime() const;

    /// \brief A signal used to set the sim time line edit.
    /// \param[in] _string String representation of sim time.
    signals: void SetTime(QString _string);

    /// \brief A signal used to set the style of the start/stop button.
    /// \param[in] _string State ("Start"/"Stop").
    signals: void SetStartStopButton(QString _state);

    /// \brief Callback that receives timer control message.
    /// \param[in] _msg "start" = start timer, "stop" = stop timer, "reset"
    /// = reset timer.
    private: void OnTimerCtrl(ConstGzStringPtr &_msg);

    /// \brief Start the timer.
    private: void Start();

    /// \brief Stop the timer.
    private: void Stop();

    /// \brief Reset the timer.
    private: void Reset();

    /// \brief Callback when the start/stop button is pressed.
    private slots: void OnStartStopButton();

    /// \brief Callback to set the start/stop button style.
    /// \param[in] _string State ("Start"/"Stop").
    private slots: void OnSetStartStopButton(QString _state);

    /// \brief Callback when the reset button is pressed.
    private slots: void OnResetButton();

    /// \brief Handles the prerender callback
    private: void PreRender();

    /// \brief Qt event filter currently used to filter resize events.
    /// \param[in] _obj Object that is watched by the event filter.
    /// \param[in] _event Qt event.
    /// \return True if the event is handled.
    private: bool eventFilter(QObject *_obj, QEvent *_event);

    /// \brief Node used to establish communication with gzserver.
    private: transport::NodePtr node;

    /// \brief Subscriber to control signals.
    private: transport::SubscriberPtr ctrlSub;

    /// \brief The actual timer
    private: common::Timer timer;

    /// \brief Set of Gazebo signal connections.
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief Mutex to protect timer updates.
    private: boost::mutex timerMutex;

    /// \brief Start/stop button.
    private: QPushButton *startStopButton;

    /// \brief QT style for the start setting of the start/stop button
    private: std::string startStyle;

    /// \brief QT style for the stop setting of the start/stop button
    private: std::string stopStyle;

    /// \brief Reset button.
    private: QPushButton *resetButton;

    /// \brief Number of pixels from the timer's left side and the
    /// window's right side. Only needed for negative positions.
    private: int posX;

    /// \brief Number of pixels from the timer's top and the window's
    /// bottom. Only needed for negative positions.
    private: int posY;
  };
}

#endif
