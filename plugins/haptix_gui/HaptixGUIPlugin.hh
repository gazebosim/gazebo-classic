/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef _GUI_ARAT_PLUGIN_HH_
#define _GUI_ARAT_PLUGIN_HH_

#include <queue>

#include <ignition/transport.hh>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>


namespace gazebo
{
  class TaskButton;

  /*
  class KeyCommand
  {
    public: KeyCommand(const char _button, const std::string &_name,
                       const float _increment) :
        button(_button), name(_name), increment(_increment) {}

    public: KeyCommand()
    {
      this->index = 0;
    }

    public: char button;
    public: std::string name;
    public: int index;
    public: float increment;
  };

  class Grasp
  {
    public: Grasp(char _incKey, char _decKey)
            : incKey(_incKey), decKey(_decKey)
    {
      this->sliderValue = 0;
    }

    public: Grasp() {}

    public: float sliderValue;
    public: std::vector<float> desiredGrasp;
    public: char incKey;
    public: char decKey;
    public: void SliderChanged(char key, float inc);
  };



  // Digital clock adapted from
  // http://qt-project.org/doc/qt-4.8/widgets-digitalclock.html
  class DigitalClock : public QLCDNumber
  {
    Q_OBJECT

    public: DigitalClock(QWidget *_parent = 0);
    public: void StopClock();

    protected slots: void ShowTime();

    private slots: void OnStartStop();

    private: QTime lastStartTime;
    private: bool running;
  };
*/



  /// \brief Make HAPTIX GUI plugin
  class HaptixGUIPlugin : public gazebo::GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: HaptixGUIPlugin();

    /// \brief Destructor
    public: virtual ~HaptixGUIPlugin();

    // Documentation inherited
    public: void Load(sdf::ElementPtr _elem);

    /// \brief Callback when a finger contact message is received.
    /// \param[in] _msg Contact message.
    private: void OnFingerContact(ConstContactsPtr &_msg);

    /// \brief Signal to set a contact visualization value.
    /// \param[in] _contactName Name of the contact sensor.
    /// \param[in] _value Force value.
    signals: void SetContactForce(QString _contactName, double _value);

    /// \brief Handles setting a contact visualization value.
    /// \param[in] _contactName Name of the contact sensor.
    /// \param[in] _value Force value.
    private slots: void OnSetContactForce(QString _contactName, double _value);

    /// \brief Handles the PreRender Gazebo signal
    private: void PreRender();

    /// \brief Callback triggered when a task button is pressed.
    /// \param[in] _id ID of the task.
    private slots: void OnTaskSent(const int _id);

    /// \brief Helper function to publish a task message
    /// \param[in] _taskName Name of the task to publish
    private: void PublishTaskMessage(const std::string &_taskName) const;














    /// \brief Callback triggered when the reset button is clicked
    protected slots: void OnResetClicked();

    /// \brief Callback triggered when the next button is clicked
    protected slots: void OnNextClicked();

    /// \brief Callback triggered when the start/stop button is clicked
    protected slots: void OnStartStopClicked();

    /// \brief Helper function to initialize the task view
    /// \param[in] _elem SDF element pointer that contains HAPTIX task
    /// parameters.
    private: void InitializeTaskView(sdf::ElementPtr _elem);

    private: bool OnKeyPress(common::KeyEvent _event);

    /// \brief Size of the contact sensor display circle, in pixels.
    private: int circleSize;

    /// \brief Minimum force value
    private: float forceMin;

    /// \brief Maximum force value
    private: float forceMax;

    /// \brief Minimum force color value
    private: common::Color colorMin;

    /// \brief Maximum force color value
    private: common::Color colorMax;

    /// \brief Which hand is displayed (left, right)
    private: std::string handSide;

    /// \brief All the finger contact points.
    private: std::map<std::string, math::Vector2d > contactPoints;

    private: virtual void mousePressEvent(QMouseEvent *_mouseEvent);

    /// \brief The scene onto which is drawn the hand and contact
    /// force data
    private: QGraphicsScene *handScene;

    /// \brief Contact force visualization items.
    private: std::map<std::string, QGraphicsEllipseItem*>
             contactGraphicsItems;

    /// \brief Subscriber to finger contact sensors.
    private: std::vector<transport::SubscriberPtr> contactSubscribers;

    /// \brief Node used to establish communication with gzserver.
    private: gazebo::transport::NodePtr node;

    // \brief Set of Gazebo signal connections.
    private: std::vector<event::ConnectionPtr> connections;

    private: QTabWidget *taskTab;

    /// \brief Text box that hold instructions to the user.
    private: QTextEdit *instructionsView;

    /// \brief All the tasks in all the groups
    private: std::map<int, TaskButton*> taskList;

    /// \brief Number of the current task.
    private: int currentTaskId;

    /// \brief Publisher that talks with the arrange plugin to setup the
    /// scene.
    private: gazebo::transport::PublisherPtr taskPub;

/*    private: ignition::transport::Node *ignNode;

    private: std::map<char, KeyCommand> armCommands;
    private: std::map<char, KeyCommand> handCommands;
    private: std::map<std::string, std::vector<char> > buttonNames;

    // The mapping between a button and the grasp it is commanding
    private: std::map<char, std::string> graspCommands;

    private: std::map<std::string, Grasp> grasps;


    /// \brief Maximum number tasks.
    /// \sa taskNum
    private: int handImgX;
    private: int handImgY;

    private: std::string handImgFilename;
    private: std::string configFilename;

    private: float GUIScaleFactor;
    private: math::Vector2d iconSize;
    private: float graspIncrement;

    private: bool isTestRunning;
    private: QString startButtonStyle;
    private: QString stopButtonStyle;




    private: QPushButton *startStopButton;

    private: DigitalClock *digitalClock;


    */
  };
}
#endif
