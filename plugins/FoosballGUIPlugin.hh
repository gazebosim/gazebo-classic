/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GUI_FOOSBALL_PLUGIN_HH_
#define _GUI_FOOSBALL_PLUGIN_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace gazebo
{
  /// \brief A GUI plugin that ...
  class GAZEBO_VISIBLE FoosballGUIPlugin : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor.
    public: FoosballGUIPlugin();

    /// \brief Destructor.
    public: virtual ~FoosballGUIPlugin();

    // Documentation inherited.
    public: void Load(sdf::ElementPtr _elem);

    /// \brief Callback when receives a time message. Updates the time display.
    /// \param[in] _time Time message.
    public: void OnTime(ConstTimePtr &_time);

    /// \brief Callback when receives a score message. Updates the score
    /// display.
    /// \param[in] _score Message containing the score string.
    public: void OnScore(ConstGzStringPtr &_score);

    /// \brief Callback when receives a state message. Updates the state
    /// display.
    /// \param[in] _state Message containing the state string.
    public: void OnState(ConstGzStringPtr &_state);

    /// \brief Publish restart game message on hotkey.
    public slots: void OnRestartGame();

    /// \brief Publish restart ball message on hotkey.
    public slots: void OnRestartBall();

    /// \brief Set time display.
    /// \param[in] _time Time string.
    signals: void SetTime(QString _time);

    /// \brief Set score display.
    /// \param[in] _score Score string.
    signals: void SetScore(QString _score);

    /// \brief Set state display.
    /// \param[in] _state State string.
    signals: void SetState(QString _state);

    /// \brief Qt event filter currently used to filter resize events.
    /// \param[in] _obj Object that is watched by the event filter.
    /// \param[in] _event Qt event.
    /// \return True if the event is handled.
    private: bool eventFilter(QObject *_obj, QEvent *_event);

    /// \brief SDF for this plugin.
    private: sdf::ElementPtr sdf;

    /// \brief Pointer to a node for communication.
    private: transport::NodePtr gzNode;

    /// \brief Time subscriber.
    private: transport::SubscriberPtr timeSub;

    /// \brief Score subscriber.
    private: transport::SubscriberPtr scoreSub;

    /// \brief State subscriber.
    private: transport::SubscriberPtr stateSub;

    /// \brief "Restart game" publisher.
    private: transport::PublisherPtr restartGamePub;

    /// \brief "Restart ball" publisher.
    private: transport::PublisherPtr restartBallPub;

    /// \brief Pointer to the render widget used to get width.
    private: QWidget *renderWidget;
  };
}

#endif
